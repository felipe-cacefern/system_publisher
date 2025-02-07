
import shutil
import psutil
import os
import time
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String



#HELPER FUNCTIONS


def remainingStorage():
    path = "/"
    total, used, free = shutil.disk_usage(path) 
    return free

def cpuUsage():
    cpuCount = os.cpu_count()
    totalUsePercent = cpuCount * psutil.cpu_percent()
    return totalUsePercent

def ramUsage():
    memory = psutil.virtual_memory()
    return memory.used

def ramCapacity():
    memory = psutil.virtual_memory()
    return memory.available

def swapUsage():
    total, used, free, percent, sin, sout = psutil.swap_memory()
    # import ipdb; ipdb.set_trace()
    print('THE USED VALUE:', used)
    print('TYPE:', type(used))
    return used

def swapChange():
    interval = 1
    initial = float(swapUsage())
    time.sleep(interval)
    final = float(swapUsage())
    return  (final - initial) / interval


def gpuUsage():
    result = subprocess.run(
        ['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total,memory.free', '--format=csv,noheader,nounits'],
        capture_output=True, text=True
    )
    
    # Parse the output
    gpu_utilization_per, memory_used_mb, memory_total_mb, _ = result.stdout.strip().split(',')

    return gpu_utilization_per, memory_used_mb, memory_total_mb



#ROS2 NODE

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.publisher_ = self.create_publisher(String, 'topic_out', 10)
        self.publisher_ = self.create_publisher(String, 'topic_out', 10)

        self.subscription = self.create_subscription(
            String,
            'topic_in',
            self.listener_callback,
            10
        )
        
        # Start a timer to publish a message at regular intervals (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)


    def correctDecimalPoints(self, n, x):
        decimalPoints = x
        n = n * 10**decimalPoints
        n //= 1
        n = n * 10**decimalPoints
        return n

    def mb_to_gb(n):
        return n / (1024 ** 3)

    def timer_callback(self):
        # Publish a message
        msg = String()

        decimals = 3

        remainingStorage_gb = self.mb_to_gb(remainingStorage())
        remainingStorage_gb = self.correctDecimalPoints(remainingStorage_gb)

        gpu_utility_per, gpu_mem_used_mb, gpu_mem_total_mb = gpuUsage()

        gpu_mem_used_gb = self.mb_to_gb(gpu_mem_used_mb)
        gpu_mem_used_gb = self.correctDecimalPoints(gpu_mem_used_gb, decimals)

        gpu_mem_total_gb = self.mb_to_gb(gpu_mem_total_mb)
        gpu_mem_total_gb = self.correctDecimalPoints(gpu_mem_total_gb, decimals)

        cpuUsage_per = cpuUsage()

        ramUsage_gb = self.mb_to_gb(ramUsage())
        ramUsage_gb = self.correctDecimalPoints(ramUsage_gb, decimals)

        ramCapacity_gb = self.mb_to_gb(ramCapacity())
        ramCapacity_gb = self.correctDecimalPoints(ramCapacity_gb, decimals)

        swapUsage_gb = self.mb_to_gb(swapUsage())
        swapUsage_gb = self.correctDecimalPoints(swapUsage_gb, decimals)

        swapChange_gb = self.mb_to_gb(swapChange())
        swapChange_gb = self.correctDecimalPoints(swapChange_gb)

        msg.data = \
            f'Remaining Storage: {remainingStorage_gb}GB\n'\
            f'CPU Usage: {cpuUsage_per}%\n'\
            f'RAM Usage: {ramUsage_gb}GB\n'\
            f'RAM Capacity: {ramCapacity_gb}GB\n'\
            f'Swap Usage: {swapUsage_gb}GB\n'\
            f'Swap Change: {swapChange_gb}GB/s\n'\
            f'GPU Usage: {gpu_utility_per}%\n'\
            f'GPU Memory Used: {gpu_mem_used_gb}GB\n'\
            f'GPU Memory Capacity: {gpu_mem_total_gb}GB'


        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def listener_callback(self, msg):
        # Handle the message received from 'topic_in'
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = MyNode()

    # Spin the node to keep it running and processing callbacks
    rclpy.spin(node)

    # Shutdown the node when done
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
