"""Awaitable look/turn helpers; the caller must keep its ROS executor spinning."""
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from simulation_actions.action import Look, LookAhead, Turn


class LookTurnClient:
    def __init__(self, node):
        self.clients = {name: ActionClient(node, kind, '/' + name)
                        for name, kind in {
                            'look_left': Look, 'look_right': Look,
                            'look_ahead': LookAhead,
                            'turn_left': Turn, 'turn_right': Turn}.items()}

    async def _send(self, name, goal):
        client = self.clients[name]
        if not client.wait_for_server(timeout_sec=5.0):
            return False
        handle = await client.send_goal_async(goal)
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success

    async def look_left(self, angle=0.0):
        return await self._send('look_left', Look.Goal(angle=float(angle)))

    async def look_right(self, angle=0.0):
        return await self._send('look_right', Look.Goal(angle=float(angle)))

    async def look_ahead(self):
        return await self._send('look_ahead', LookAhead.Goal())

    async def turn_left(self, angle=0.0):
        return await self._send('turn_left', Turn.Goal(angle=float(angle)))

    async def turn_right(self, angle=0.0):
        return await self._send('turn_right', Turn.Goal(angle=float(angle)))

    def destroy(self):
        for client in self.clients.values():
            client.destroy()
