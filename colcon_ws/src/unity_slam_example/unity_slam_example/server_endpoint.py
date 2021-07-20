import rclpy

from ros2_tcp_endpoint.server import TcpServer
from ros2_tcp_endpoint.publisher import RosPublisher
from ros2_tcp_endpoint.subscriber import RosSubscriber
from ros2_tcp_endpoint.service import RosService
from ros2_tcp_endpoint.unity_service import UnityService

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)

    ros_node_name = 'TCPServer'
    buffer_size = 1024
    connections = 10
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)

    tcp_server.source_destination_dict = {
        'scan': RosPublisher('scan', LaserScan, queue_size=10),
        'clock': RosPublisher('clock', Clock, queue_size=10),
        'tf': RosPublisher('tf', TFMessage, queue_size=10),
        'cmd_vel': RosSubscriber('cmd_vel', Twist, tcp_server),
    }
    tcp_server.start()

    # Setup executors for nodes defined in source_destination_dict
    tcp_server.setup_executor()

    # Clean up nodes defined in source_destination_dict
    tcp_server.destroy_nodes()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
