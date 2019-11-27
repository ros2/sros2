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

import sys

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class ClientServiceNode(Node):

    def __init__(self):
        super().__init__('test_generate_policy_services_node')
        self.client = self.create_client(Trigger, 'test_generate_policy_services_client')
        self.service = self.create_service(
            Trigger, 'test_generate_policy_services_server', lambda request, response: response)

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
    except BaseException:
        print('exception in node:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
