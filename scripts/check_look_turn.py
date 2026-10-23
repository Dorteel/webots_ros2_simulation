#!/usr/bin/env python3
"""Live basic-world check: five actions, head/base isolation and cancellation."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from simulation_actions.action import Look, LookAhead, Turn


def main():
    rclpy.init()
    node = rclpy.create_node('look_turn_live_check')
    latest = {}
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, key=key: latest.update({key: msg}), qos_profile_sensor_data)
        for key, kind, topic in [('odom', Odometry, '/odom'),
                                 ('joints', JointState, '/joint_states')]]
    types = {'look_left': Look, 'look_right': Look, 'look_ahead': LookAhead,
             'turn_left': Turn, 'turn_right': Turn}
    clients = {name: ActionClient(node, kind, '/'+name) for name, kind in types.items()}
    active = None

    def wait(condition, timeout=10):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not condition():
            raise RuntimeError('Timed out waiting for live look/turn state')

    def future_value(future, timeout=15):
        wait(future.done, timeout)
        return future.result()

    def snapshot():
        joints = dict(zip(latest['joints'].name, latest['joints'].position))
        odom = latest['odom']
        q = odom.pose.pose.orientation
        return (joints['head_1_joint'], joints['head_2_joint'],
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def run(name, angle=None, cancel=False):
        nonlocal active
        goal = types[name].Goal()
        if angle is not None:
            goal.angle = angle
        active = future_value(clients[name].send_goal_async(goal))
        if not active.accepted:
            raise RuntimeError(f'{name} rejected')
        result_future = active.get_result_async()
        if cancel:
            response = future_value(active.cancel_goal_async())
            assert response.goals_canceling, 'Cancellation rejected'
        response = future_value(result_future, 45)
        active = None
        assert response.status == (GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_SUCCEEDED), response
        # Allow fresh feedback after Supervisor scene writes and wheel stop.
        until = time.monotonic()+0.2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        return response.result

    try:
        for client in clients.values():
            assert client.wait_for_server(timeout_sec=10), 'Missing look/turn action'
        wait(lambda: 'odom' in latest and 'joints' in latest)
        old = {'/pick', '/place', '/place_to_object', '/place_next_to', '/place_in_container', '/open', '/close'}
        from rclpy.action import get_action_names_and_types
        wait(lambda: old <= {name for name, _ in get_action_names_and_types(node)})
        run('look_ahead')
        initial = snapshot()
        for name in ('look_left', 'look_right'):
            before = snapshot()
            result = run(name)
            after = snapshot()
            sign = 1 if name == 'look_left' else -1
            assert sign*(after[0]-before[0]) > 0.3, 'Wrong head direction/default'
            assert abs(result.achieved_angle-math.pi/6) < 0.03
            assert math.hypot(after[2]-before[2], after[3]-before[3]) < 0.03
            assert abs(after[4]-before[4]) < 0.03, 'Head action rotated base'
        run('look_left', 0.2)
        assert abs(snapshot()[0]-initial[0]-0.2) < 0.03
        run('look_ahead')
        assert max(abs(value) for value in snapshot()[:2]) < 0.02
        for name in ('turn_left', 'turn_right'):
            before = snapshot()
            result = run(name)
            after = snapshot()
            assert abs(result.achieved_angle-math.pi/2) < 0.06
            assert max(abs(after[i]-before[i]) for i in (0, 1)) < 0.02, 'Turn moved head'
            delta = math.atan2(math.sin(after[4]-before[4]), math.cos(after[4]-before[4]))
            assert delta*(1 if name == 'turn_left' else -1) > 1.4, 'Wrong turn sign'
        result = run('turn_right', 0.3)
        assert abs(result.achieved_angle-0.3) < 0.06
        run('turn_left', 2.0, cancel=True)
        wait(lambda: abs(latest['odom'].twist.twist.angular.z) < 0.03, 5)
        print('PASS: five actions, defaults/custom angles, head/base isolation, neutral, signs, cancel/stop, legacy discovery')
    finally:
        if active is not None:
            try:
                future_value(active.cancel_goal_async(), 3)
            except Exception:
                pass
        for client in clients.values():
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
