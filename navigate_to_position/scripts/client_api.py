"""Awaitable client for the existing /navigate_to_position action."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from navigate_to_position.action import NavigateToPosition


class NavigateToPositionClient:
    def __init__(self, node):
        self._client = ActionClient(node, NavigateToPosition, '/navigate_to_position')

    async def navigate(self, x, y, yaw, frame_id='map'):
        """Return success; the caller must keep its ROS executor spinning."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        goal = NavigateToPosition.Goal(x=x, y=y, yaw=yaw, frame_id=frame_id)
        handle = await self._client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
