import os
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from navigation_interfaces.srv import Navigation

from navigation_server.PPO import PPO


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
state_dim = 11
action_dim = 2
action_std = 1.
lr = 0.0003


class NavigationServerNode(Node):
    def __init__(self):
        super().__init__('navigation_server_node')
        self.srv = self.create_service(Navigation, 'navigation', self.navigation_callback)
        
        package_share_dir = get_package_share_directory('navigation_server')


        self.scout_model = PPO(state_dim, action_dim, lr, action_std, [0], hidden_size=128, inference=True)
        self.scout_model.load('navigation.pth', os.path.join(package_share_dir, 'weights'))

        print("Finished init...")
 
    def navigation_callback(self, request, response):
        self.obs = np.concatenate(([request.angle], [request.speed], [request.distance], request.front_lidar, request.rear_lidar))
        action = self.scout_model.select_action(0, torch.tensor(self.obs, dtype=torch.float32).to(device))
         
        response.torque = float(action[0])
        response.steering = float(action[1])
        return response
 
 
def main(args=None):
    rclpy.init(args=args)
    navigation_server = NavigationServerNode()
    rclpy.spin(navigation_server)
    navigation_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()