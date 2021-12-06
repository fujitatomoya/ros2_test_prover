import rclpy
import std_msgs.msg as msgs
import time
import asyncio


def callback(msg):
    # never gets called
   pass


async def sub_task(node):
    time.sleep(1)
    sub = node.create_subscription(msgs.Float32, "/topic", callback, 10)
    time.sleep(5)

async def main():

    rclpy.init()
    node = rclpy.create_node("new_node")
    task = asyncio.create_task(sub_task(node))
    #await task
    rclpy.spin(node)
    pass

asyncio.run(main())
