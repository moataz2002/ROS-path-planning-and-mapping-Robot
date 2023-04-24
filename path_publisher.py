#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_planning import astar, grid, start, end

def publish_path():
    rospy.init_node('path_publisher', anonymous=True)
    path_publisher = rospy.Publisher('/robot_path', Path, queue_size=1)

    rate = rospy.Rate(1)

   path = astar(grid, start, end)

    while not rospy.is_shutdown():
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        path_publisher.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
