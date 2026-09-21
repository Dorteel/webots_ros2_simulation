"""ROS adapters for existing Supervisor place/open/close actions."""

from rclpy.action import ActionServer
from simulation_actions.action import Close, Open, Place, PlaceToObject

from actions import execute_action


ACTION_TYPES = {
    'place': Place,
    'place_to_object': PlaceToObject,
    'open': Open,
    'close': Close,
}


class FallbackActionServers:
    def __init__(self, ros_node, supervisor):
        self.servers = [
            ActionServer(
                ros_node, action_type, f'/{name}',
                execute_callback=lambda goal, name=name, action_type=action_type:
                    self.execute(supervisor, goal, name, action_type),
            )
            for name, action_type in ACTION_TYPES.items()
        ]

    @staticmethod
    def execute(supervisor, goal_handle, name, action_type):
        request = goal_handle.request
        parameters = {'robot': request.robot, 'object': request.object}
        if name == 'place':
            parameters['coordinates'] = [request.x, request.y, request.z]
        elif name == 'place_to_object':
            parameters['target'] = request.target

        feedback = action_type.Feedback()
        feedback.status = f'{name} in progress'
        goal_handle.publish_feedback(feedback)
        result = action_type.Result()
        try:
            execute_action(supervisor, name, parameters)
        except (ValueError, TypeError) as error:
            result.message = str(error)
            goal_handle.abort()
        else:
            result.success = True
            result.message = f'{name} completed'
            goal_handle.succeed()
        return result

    def destroy(self):
        for server in self.servers:
            server.destroy()
