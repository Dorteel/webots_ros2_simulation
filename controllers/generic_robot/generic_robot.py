"""Per-robot hardware inventory, measured joint feedback and ordinary motor commands.

Loaded by webots_ros2_driver, which owns stepping and native sensor plugins.
"""
import json
import math
import os
import re

from controller import Motor, PositionSensor
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


NATIVE_SENSORS = {'Camera', 'RangeFinder', 'Lidar', 'DistanceSensor', 'LightSensor',
                  'GPS', 'Compass', 'Receiver'}


class GenericRobot:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.namespace = json.loads(os.environ['GENERIC_ROBOT_NAMESPACES'])[self.robot.getName()]
        rclpy.init(args=[])
        self.node = rclpy.create_node('hardware', namespace=self.namespace)
        self.motors = {}
        self.sensors = []
        self.subscriptions = []
        self.velocity_deadlines = {}
        self.period = int(self.robot.getBasicTimeStep())
        self.position_speed = float(properties.get('positionSpeed', '0.5'))
        self.command_timeout = float(properties.get('velocityTimeout', '0.5'))
        if not all(math.isfinite(x) and x > 0 for x in (self.position_speed, self.command_timeout)):
            raise ValueError('positionSpeed and velocityTimeout must be finite and positive')
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inventory = self.node.create_publisher(String, 'hardware_info', retained)
        self.states = self.node.create_publisher(JointState, 'joint_states', 10)
        self.status = self.node.create_publisher(String, 'command_status', 10)
        devices = [self.robot.getDeviceByIndex(i) for i in range(self.robot.getNumberOfDevices())]
        sensor_joints = {}
        descriptions = []
        motor_ids = set()
        for device in devices:
            name = device.getName()
            entry = {'name': name, 'type': type(device).__name__, 'observable': type(device).__name__ in NATIVE_SENSORS,
                     'command_topics': {}}
            if isinstance(device, Motor):
                base = re.sub(r'[^a-z0-9_]', '_', name.lower()) or 'motor'
                if base[0].isdigit():
                    base = 'motor_' + base
                motor_id = base
                suffix = 2
                while motor_id in motor_ids:
                    motor_id = f'{base}_{suffix}'
                    suffix += 1
                motor_ids.add(motor_id)
                entry['motor_id'] = motor_id
                sensor = device.getPositionSensor()
                if sensor is not None:
                    sensor_joints[sensor.getName()] = name
                lower, upper = device.getMinPosition(), device.getMaxPosition()
                maximum = device.getMaxVelocity()
                bounded = lower != upper
                # Coupled axes require an ownership policy before independent commands.
                supported = ('::' not in name and device.getMultiplier() == 1.0
                             and math.isfinite(maximum) and maximum > 0
                             and math.isfinite(lower) and math.isfinite(upper)
                             and (not bounded or lower < upper))
                entry.update(position_sensor=sensor.getName() if sensor else None,
                             observable=sensor is not None,
                             unit='rad' if device.getType() == Motor.ROTATIONAL else 'm',
                             min_position=lower if math.isfinite(lower) else None,
                             max_position=upper if math.isfinite(upper) else None,
                             position_bounded=bounded,
                             max_velocity=maximum if math.isfinite(maximum) else None)
                if supported:
                    self.motors[name] = (device, lower, upper, maximum)
                    # Continuous velocity mode is offered only for unbounded motors.
                    modes = ('position',) if bounded else ('position', 'velocity')
                    for mode in modes:
                        topic = f'motors/{motor_id}/{mode}'
                        entry['command_topics'][mode] = '/' + self.namespace + '/' + topic
                        self.subscriptions.append(self.node.create_subscription(
                            Float64, topic, lambda msg, n=name, m=mode: self.command(n, m, msg.data), 1))
                else:
                    entry['command_limitation'] = 'Coupled motor or invalid/unsupported limits'
            elif isinstance(device, PositionSensor):
                entry['observable'] = True
            else:
                entry['exposure'] = ('native_driver' if type(device).__name__ in NATIVE_SENSORS
                                     else 'inventory_only_or_native_driver; inspect ROS topics')
            descriptions.append(entry)
        for device in devices:
            if isinstance(device, PositionSensor):
                device.enable(self.period)
                self.sensors.append((sensor_joints.get(device.getName(), device.getName()), device))
        # Disambiguate passive sensor names that equal another motor's joint name.
        names = [name for name, _ in self.sensors]
        used = {name for name in names if names.count(name) == 1}
        for index, (name, sensor) in enumerate(self.sensors):
            if names.count(name) > 1:
                candidate = f'{sensor.getName()}_sensor_{index + 1}'
                while candidate in used:
                    candidate += '_'
                used.add(candidate)
                self.sensors[index] = (candidate, sensor)
        info = {'schema_version': 1, 'robot_name': self.robot.getName(),
                'namespace': '/' + self.namespace, 'devices': descriptions,
                'joint_feedback': [{'joint_name': name, 'position_sensor': sensor.getName()}
                                   for name, sensor in self.sensors],
                'position_command_speed_cap': self.position_speed,
                'velocity_timeout_sim_seconds': self.command_timeout}
        self.inventory.publish(String(data=json.dumps(info, allow_nan=False, sort_keys=True)))
        self.node.get_logger().info(f'Discovered {len(devices)} devices, {len(self.motors)} commandable motors; no startup motor commands')

    def command(self, name, mode, value):
        motor, lower, upper, maximum = self.motors[name]
        reason = None
        if not math.isfinite(value):
            reason = 'Command must be finite'
        elif mode == 'position' and lower != upper and not lower <= value <= upper:
            reason = f'Position outside [{lower}, {upper}]'
        elif mode == 'velocity' and abs(value) > maximum:
            reason = f'Velocity exceeds {maximum}'
        if reason is not None:
            self.status.publish(String(data=json.dumps({'motor': name, 'mode': mode,
                                                        'accepted': False, 'reason': reason})))
            return
        if mode == 'position':
            motor.setVelocity(min(maximum, self.position_speed))
            motor.setPosition(value)
            self.velocity_deadlines.pop(name, None)
        else:
            motor.setPosition(float('inf'))
            motor.setVelocity(value)
            self.velocity_deadlines[name] = self.robot.getTime() + self.command_timeout
        self.status.publish(String(data=json.dumps({'motor': name, 'mode': mode,
                                                    'accepted': True, 'value': value})))

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        now = self.robot.getTime()
        for name, deadline in list(self.velocity_deadlines.items()):
            if now >= deadline:
                self.motors[name][0].setVelocity(0.0)
                del self.velocity_deadlines[name]
                self.status.publish(String(data=json.dumps({'motor': name, 'event': 'velocity_timeout'})))
        message = JointState()
        message.header.stamp = Time(nanoseconds=round(now * 1e9)).to_msg()
        for name, sensor in self.sensors:
            position = sensor.getValue()
            if math.isfinite(position):
                message.name.append(name)
                message.position.append(position)
        if message.name:
            self.states.publish(message)

    def stop(self):
        for name in self.velocity_deadlines:
            self.motors[name][0].setVelocity(0.0)
        self.node.destroy_node()
        rclpy.shutdown()
