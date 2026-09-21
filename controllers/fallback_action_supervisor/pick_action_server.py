"""Thin ROS action adapter for the Supervisor's existing pick implementation."""

from rclpy.action import ActionServer
from simulation_actions.action import Pick

from actions import execute_action


class PickActionServer:
    def __init__(self, ros_node, supervisor):
        self.server = ActionServer(
            ros_node, Pick, '/pick',
            execute_callback=lambda goal: self.execute(supervisor, goal),
        )

    @staticmethod
    def execute(supervisor, goal_handle):
        result = Pick.Result()
        feedback = Pick.Feedback()
        feedback.status = 'Picking object'
        goal_handle.publish_feedback(feedback)
        try:
            execute_action(supervisor, 'pick', {
                'robot': goal_handle.request.robot,
                'object': goal_handle.request.object,
            })
        except (ValueError, TypeError) as error:
            result.message = str(error)
            goal_handle.abort()
        else:
            result.success = True
            result.message = 'Picked object'
            goal_handle.succeed()
        return result

    def destroy(self):
        self.server.destroy()
