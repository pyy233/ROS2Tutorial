from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_pub = Node(
        package="example_topic_rclcpp",
        executable="topic_publisher_01"
    )
    node_sub = Node(
        package="example_topic_rclcpp",
        executable="topic_subscribe_01"
    )
    
    launch_description = LaunchDescription(
        [node_pub, node_sub])
    # 返回让ROS2根据launch描述执行节点
    return launch_description