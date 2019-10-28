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

import test_msgs.msg


class PubSubNode(Node):

    def __init__(self, name='pub_sub_node'):
        super().__init__(name)

        self.publisher = self.create_publisher(
            test_msgs.msg.Strings, '~/pub', 1
        )
        self.subscription = self.create_subscription(
            test_msgs.msg.Strings, '~/sub', lambda msg: None, 1
        )

    def destroy_node(self):
        self.publisher.destroy()
        self.subscription.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PubSubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
