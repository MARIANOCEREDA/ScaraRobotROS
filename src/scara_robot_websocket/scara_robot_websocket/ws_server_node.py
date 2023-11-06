import rclpy
from rclpy.node import Node
import asyncio
from example_interfaces.msg import Float64MultiArray
from websockets.server import serve
import json

TOPIC_NAME = "/robot_position"
NODE_NAME = "ws_server_node"

class WsServerNode(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info("Node " + NODE_NAME + " started.")
        self.robot_position_pub = self.create_publisher(Float64MultiArray, TOPIC_NAME, 10)
        asyncio.run(self.run_ws())

    def convert_to_multiarray(self, input_msg:str) -> Float64MultiArray:

        new_msg = Float64MultiArray()
        key_value_pairs = input_msg[1:-1].split(',')

        result_dict = {}
        for pair in key_value_pairs:
            key, value = pair.split(':')
            new_msg.data.append(float(value))

        return new_msg

    async def echo(self, websocket):

        async for message in websocket:

            self.get_logger().info("Message position from ws client: " + message)

            converted_msg = self.convert_to_multiarray(message)

            self.robot_position_pub.publish(converted_msg)

            await websocket.send(message)
    
    async def run_ws(self):
        async with serve(self.echo, "localhost", 8765):
            await asyncio.Future() 


def main(arg=None):
    rclpy.init(args=arg)
    node = WsServerNode()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()