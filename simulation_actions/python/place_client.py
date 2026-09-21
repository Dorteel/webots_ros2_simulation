"""Awaitable client for the Supervisor-backed /place action."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from simulation_actions.action import Place


class PlaceClient:
    def __init__(self, node):
        self._client = ActionClient(node, Place, '/place')

    async def place(self, robot, object, x, y, z):
        """Return success; the caller must keep its ROS executor spinning."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        goal = Place.Goal(robot=robot, object=object, x=x, y=y, z=z)
        handle = await self._client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
