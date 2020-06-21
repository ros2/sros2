# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import test_msgs.srv


class ClientServiceNode(Node):

    def __init__(self, name='client_srv_node'):
        super().__init__(name)
        self.client = self.create_client(test_msgs.srv.Empty, '~/client')
        self.service = self.create_service(
            test_msgs.srv.Empty, '~/server', lambda request, response: response
        )

    def destroy_node(self):
        self.client.destroy()
        self.service.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ClientServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
