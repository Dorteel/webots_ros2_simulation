"""Awaitable client for the Supervisor-backed /close action."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from simulation_actions.action import Close


class CloseClient:
    def __init__(self, node):
        self._client = ActionClient(node, Close, '/close')

    async def close(self, robot, object):
        """Return success; the caller must keep its ROS executor spinning."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        goal = Close.Goal(robot=robot, object=object)
        handle = await self._client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
