#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


#########
# get all the obstacles
# define the size of the bot
# for each obstacle
    # grow it
    # create marker
    # publish the marker to ros
##########

# parse for obstacles
def get_obstacles(path):
    with open(path) as f:
        lines = f.readlines()
        lines = [x.strip() for x in lines] 
    
    # how to return the obstacles?



# make a line marker
# will inform how we get the obstacles
def publish_marker():
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    # need some time for ppl to sub to marker pub
    rospy.sleep(0.5)

    marker = Marker(
                type=Marker.LINE_STRIP,
                id=0,
                lifetime=rospy.Duration(1.5),   # 0 means forever
                # pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                # scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                points=[Point(1,1,1), Point(2,2,1)])

    
    marker_publisher.publish(marker)
    return


def main():
  rospy.init_node('create_hull', anonymous=False)
  publish_marker()

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("create-hull node terminated.")