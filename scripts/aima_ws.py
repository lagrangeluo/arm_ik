import asyncio
import websockets
import json
import time

# 新增导入
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class RobotArmWebSocketClient():
    def __init__(self, host="192.168.100.100", port=9002):
        self.uri = f"ws://{host}:{port}"
        self.ws = None
        # ROS2节点和发布者初始化为None
        self.ros_node = None
        self.gripper_pub = None
        self.last_cmd_list = None

        rclpy.init()
        self.ros_node = Node('gripper_cmd_publisher')
        self.gripper_pub = self.ros_node.create_publisher(Int32MultiArray, '/agi_hw/gripper_control', 10)
        print("init success")


    #----------- ROS2夹爪发布 -----------#
    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = Node('gripper_cmd_publisher')
        self.gripper_pub = self.ros_node.create_publisher(Int32MultiArray, '/agi_hw/gripper_control', 10)
        time.sleep(1)
        self.ros_node.get_logger().info("ros init")


    def publish_gripper(self, data):
        if self.ros_node is None or self.gripper_pub is None:
            self.init_ros()
        msg = Int32MultiArray()
        msg.data = data
        self.gripper_pub.publish(msg)
        print(f"已发布夹爪命令: {data}")

    def spin_once(self):
        if self.ros_node is not None:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    #----------- 应用层 -----------#

    async def moveL(self,cmd_x,cmd_y,cmd_z,rotate_state='0du'):
        if rotate_state=='0du':
            await self.move_arm(right_cmd_list = [cmd_x, cmd_y, cmd_z, 0.707, 0, 0.707, 0])
        elif rotate_state=='90du':
            await self.move_arm(right_cmd_list = [cmd_x, cmd_y, cmd_z, -0.5, 0.5, -0.5, 0.5])
        elif rotate_state=='45du_right':
            await self.move_arm(right_cmd_list = [cmd_x, cmd_y, cmd_z, 0.653, 0.271, 0.653, 0.271])
        elif rotate_state=='45du_left':
            await self.move_arm(right_cmd_list = [cmd_x, cmd_y, cmd_z, -0.653, 0.271, -0.653, 0.271])
        else:
            self.ros_node.get_logger().warn("Wrong rotate state!!! abord moveL")
            return
        
        self.last_cmd_list=[cmd_x,cmd_y,cmd_z,rotate_state]


    async def moveL_delta(self,cmd_x,cmd_y,cmd_z):
        if self.last_cmd_list==None:
            self.ros_node.get_logger().warn("moveL_delta failed for last_cmd_list is None")
        cmd_x = cmd_x + self.last_cmd_list[0]
        cmd_y = cmd_y + self.last_cmd_list[1]
        cmd_z = cmd_z + self.last_cmd_list[2]
        cmd_rotate = self.last_cmd_list[3]
        await self.moveL(cmd_x,cmd_y,cmd_z,rotate_state=cmd_rotate)

    async def move_rotate(self,rotate_state):
        if self.last_cmd_list==None:
            self.ros_node.get_logger().warn("moveL_delta failed for last_cmd_list is None")
        cmd_x = self.last_cmd_list[0]
        cmd_y = self.last_cmd_list[1]
        cmd_z = self.last_cmd_list[2]
        cmd_rotate = rotate_state
        await self.moveL(cmd_x,cmd_y,cmd_z,rotate_state=cmd_rotate)


    async def demo_move(self):

        self.close_gripper()
        await self.moveL(0.8, -0.35, 0.9,'0du')
        time.sleep(2)
        await self.moveL_delta(-0.1,0,0)
        time.sleep(2)
        await self.moveL_delta(0,0.1,0)
        time.sleep(2)
        await self.move_rotate('45du_right')
        time.sleep(2)
        await self.moveL_delta(-0.05,0,0)
        time.sleep(2)
        await self.move_rotate('45du_left')

    async def move_arm(self,left_cmd_list=None,right_cmd_list=None):
        left = None
        right = None

        if left_cmd_list is not None:
            left={
                "position": {"x": left_cmd_list[0], "y": left_cmd_list[1], "z": left_cmd_list[2]},
                "orientation": {"x": left_cmd_list[3], "y": left_cmd_list[4], "z": left_cmd_list[5], "w": left_cmd_list[6]}
            }
        if right_cmd_list is not None:
            right={
                "position": {"x": right_cmd_list[0], "y": right_cmd_list[1], "z": right_cmd_list[2]},
                "orientation": {"x": right_cmd_list[3], "y": right_cmd_list[4], "z": right_cmd_list[5], "w": right_cmd_list[6]}
            }

        await self.move_se3(left,right)

    def open_gripper(self):
        time.sleep(0.5)
        self.publish_gripper([2, 2])
        self.spin_once()
    def close_gripper(self):
        time.sleep(0.5)
        self.publish_gripper([1, 1])
        self.spin_once()
    #----------- 应用层-----------#

    def sync(self, coro):
        """
        同步执行任意异步协程（async函数），自动处理事件循环。
        用法：self.sync(self.moveL(...))
        """
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop and loop.is_running():
            # 如果已经有事件循环（如Jupyter），用nest_asyncio兼容
            # import nest_asyncio
            # nest_asyncio.apply()
            return loop.run_until_complete(coro)
        else:
            return asyncio.run(coro)

    #----------- websocket 服务-----------#
    async def connect(self):
        self.ws = await websockets.connect(self.uri)

    async def close(self):
        if self.ws:
            await self.ws.close()

    async def move_se3(self, left=None, right=None):
        """
        left/right: dict, 例如
        {
            "position": {"x": 0.8, "y": 0.2, "z": 1.2},
            "orientation": {"x": 0.5, "y": -0.5, "z": 0.5, "w": -0.5}
        }
        """
        msg = {"type": "planning_move"}
        if left:
            msg["left"] = left
        if right:
            msg["right"] = right
        await self.ws.send(json.dumps(msg))
        # 可选：等待服务器响应
        # resp = await self.ws.recv()
        # print("服务器响应:", resp)
    #----------- websocket 服务-----------#

    # 你可以继续扩展更多方法，如move_joint、claw_control等

async def get_aima_instance():
    client = RobotArmWebSocketClient("192.168.100.100", 9002)
    await client.connect()
    return client

# 使用示例
def main():
    # 发布夹爪命令（如需等待ROS2网络初始化，可sleep 1秒）
    #node=await get_aima_instance()
    client = RobotArmWebSocketClient()
    client.sync(client.connect())
    time.sleep(1)
    client.sync(client.demo_move())
    # await node.demo_move()
    # await client.demo_move()
    #client.sync(client.close())
    # await node.close()

if __name__ == "__main__":
    # asyncio.run(main())
    main()