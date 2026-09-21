"""Awaitable client for the Supervisor-backed /place_next_to action."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from simulation_actions.action import PlaceNextTo


class PlaceNextToClient:
    def __init__(self, node):
        self._client = ActionClient(node, PlaceNextTo, '/place_next_to')

    async def place_next_to(self, robot, object, target):
        """Return success; the caller must keep its ROS executor spinning."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        goal = PlaceNextTo.Goal(robot=robot, object=object, target=target)
        handle = await self._client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
