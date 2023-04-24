#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_planning import astar, grid, start, end

def publish_path():
    # Initialize the ROS node and the path publisher
    rospy.init_node('path_publisher', anonymous=True)
    path_publisher = rospy.Publisher('/robot_path', Path, queue_size=1)

    # Set the publishing rate (1 Hz in this example)
    rate = rospy.Rate(1)

    # Compute the path using the A* algorithm
    path = astar(grid, start, end)

    # Continuously publish the path until the node is shut down
    while not rospy.is_shutdown():
        # Create a new Path message
        path_msg = Path()

        # Set the frame ID and timestamp of the message header
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Convert each point in the path to a PoseStamped message and add it to the Path message
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        # Publish the Path message
        path_publisher.publish(path_msg)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        # Call the publish_path function
        publish_path()
    except rospy.ROSInterruptException:
        pass
