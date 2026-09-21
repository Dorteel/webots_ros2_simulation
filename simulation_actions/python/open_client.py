"""Awaitable client for the Supervisor-backed /open action."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from simulation_actions.action import Open


class OpenClient:
    def __init__(self, node):
        self._client = ActionClient(node, Open, '/open')

    async def open(self, robot, object):
        """Return success; the caller must keep its ROS executor spinning."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        goal = Open.Goal(robot=robot, object=object)
        handle = await self._client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
