#!/usr/bin/env python3
"""Ask a selected VLM about one snapshot of TIAGo's latest RGB frame."""

import base64
import json
import os
from threading import Lock
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from simulation_actions.action import ObserveWithVLM


class ObserveWithVLMActionServer(Node):
    def __init__(self):
        super().__init__('observe_with_vlm_server')
        for name, default in {
            'backend': 'ollama',
            'camera_topic': '/tiago/camera/color/image_raw',
            'ollama_url': 'http://localhost:11434',
            'ollama_model': 'qwen3-vl:2b',
            'nebula_url': 'https://nebula.cs.vu.nl/api/chat/completions',
            'nebula_model': 'SURF.Qwen3.5 122B A10B NVFP4',
            'nebula_api_key_env': 'NEBULA_API_KEY',
            'request_timeout_sec': 120.0,
        }.items():
            self.declare_parameter(name, default)
        self._latest_image = None
        self._image_lock = Lock()
        self._goal_lock = Lock()
        self._bridge = CvBridge()
        # Keep receiving frames while the action's separate callback group waits
        # for HTTP. Only the latest message is retained; no per-goal subscription.
        self._camera_callbacks = MutuallyExclusiveCallbackGroup()
        self._subscription = self.create_subscription(
            Image, self.get_parameter('camera_topic').value, self._cache_image,
            qos_profile_sensor_data, callback_group=self._camera_callbacks)
        self._server = ActionServer(
            self, ObserveWithVLM, '/observe_with_vlm', self.execute,
            goal_callback=self._accept_goal)

    def _accept_goal(self, _request):
        # One inference at a time keeps an executor thread free for the camera.
        if self._goal_lock.acquire(blocking=False):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def _cache_image(self, image):
        with self._image_lock:
            self._latest_image = image

    def _post(self, url, payload, api_key=None):
        headers = {'Content-Type': 'application/json'}
        if api_key:
            headers['Authorization'] = f'Bearer {api_key}'
        timeout = float(self.get_parameter('request_timeout_sec').value)
        if timeout <= 0:
            raise ValueError('request_timeout_sec must be positive.')
        request = Request(url, data=json.dumps(payload).encode('utf-8'),
                          headers=headers, method='POST')
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as error:
            # Do not echo remote bodies or request headers: they may contain secrets.
            raise RuntimeError(f'VLM endpoint returned HTTP {error.code}.') from None
        except (URLError, TimeoutError, OSError):
            raise RuntimeError('VLM endpoint unavailable or request timed out.') from None

    def _ask_ollama(self, prompt, image_b64):
        result = self._post(
            self.get_parameter('ollama_url').value.rstrip('/') + '/api/chat',
            {'model': self.get_parameter('ollama_model').value,
             'stream': False,
             'messages': [{'role': 'user', 'content': prompt, 'images': [image_b64]}]})
        return result['message']['content']

    def _ask_nebula(self, prompt, image_b64):
        api_key = os.environ.get(self.get_parameter('nebula_api_key_env').value)
        if not api_key:
            raise ValueError('Nebula API key is missing; set the environment variable '
                             'named by nebula_api_key_env before starting the server.')
        result = self._post(
            self.get_parameter('nebula_url').value,
            {'model': self.get_parameter('nebula_model').value,
             'stream': False,
             'messages': [{'role': 'user', 'content': [
                 {'type': 'text', 'text': prompt},
                 {'type': 'image_url', 'image_url': {
                     'url': 'data:image/jpeg;base64,' + image_b64}},
             ]}]}, api_key=api_key)
        return result['choices'][0]['message']['content']

    def execute(self, goal_handle):
        try:
            result = ObserveWithVLM.Result()
            # Snapshot the reference once. Subscription callbacks replace the cached
            # message, never mutate it, so this goal always uses exactly one frame.
            with self._image_lock:
                image = self._latest_image
            if image is None:
                result.response = 'No camera frame available.'
                goal_handle.abort()
                return result
            try:
                backend = self.get_parameter('backend').value
                if backend not in ('ollama', 'nebula'):
                    raise ValueError('Unsupported backend; expected ollama or nebula.')
                goal_handle.publish_feedback(ObserveWithVLM.Feedback(state='running_vlm'))
                # ROS bgra8 (or another cv_bridge-supported color encoding) -> BGR
                # pixels -> JPEG bytes -> base64 for the selected VLM HTTP endpoint.
                pixels = self._bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                ok, jpeg = cv2.imencode('.jpg', pixels)
                if not ok:
                    raise ValueError('Could not encode camera frame as JPEG.')
                image_b64 = base64.b64encode(jpeg.tobytes()).decode('ascii')
                ask = self._ask_ollama if backend == 'ollama' else self._ask_nebula
                response = ask(goal_handle.request.prompt, image_b64)
                if not isinstance(response, str) or not response.strip():
                    raise ValueError('VLM returned an empty or non-text response.')
                result.response = response
            except (ValueError, RuntimeError) as error:
                result.response = str(error)
            except (KeyError, IndexError, TypeError):
                result.response = 'VLM returned an invalid response format.'
            except Exception as error:
                # Conversion and transport failures must terminate the goal cleanly.
                result.response = f'Observation failed ({type(error).__name__}).'
            else:
                result.success = True
                goal_handle.succeed()
                return result
            goal_handle.abort()
            return result
        finally:
            self._goal_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = ObserveWithVLMActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
