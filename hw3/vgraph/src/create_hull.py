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


class CreateMarkers():
    def __init__(self):
        rospy.init_node('create_hull', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.init_markers()

        # now create the hulls and add the points to the marker points
        self.create_hulls()

        # publish the markers
        self.publish()

    def publish(self):
        while not rospy.is_shutdown():
            # Update the marker display
            self.marker_pub.publish(self.markers)
            rospy.sleep(0.5)


    def create_hulls(self):
        p1 = Point()
        p1.x = 0.9
        p1.y = 0
        p1.z = 0.01

        p2 = Point()
        p2.x = 0.9
        p2.y = 0.5
        p2.z = 0.01

        self.markers.points.append(p1)
        self.markers.points.append(p2)


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.02
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'hulls'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.LINE_LIST
        # self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()


    def shutdown(self):
        rospy.loginfo("Shutting down")
        rospy.sleep(2)


# # parse for obstacles
# def get_obstacles(path):
#     with open(path) as f:
#         lines = f.readlines()
#         lines = [x.strip() for x in lines] 
    
    # how to return the obstacles?



# make a line marker
# will inform how we get the obstacles
def publish_marker():
    marker_publisher = rospy.Publisher('vgraph_markers', Marker, queue_size=10)
    # need some time for ppl to sub to marker pub
    rospy.sleep(0.5)

    while not rospy.is_shutdown():

        p1 = Point()
        p1.x = 0.9
        p1.y = 0
        p1.z = 0.01

        p2 = Point()
        p2.x = 0.9
        p2.y = 0.5
        p2.z = 0.01

        points = [p1,p2]
        marker = Marker(
                    type=Marker.LINE_LIST,
                    id=0,
                    lifetime=rospy.Duration(100),   # 0 means forever
                    pose=Pose(Point(1.0, 1.0, 1.0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(1, 0.06, 0.06),
                    header=Header(frame_id='base_link'),
                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                    points=points)

        
        marker_publisher.publish(marker)

        rospy.sleep(15)

    return


# def main():
#   rospy.init_node('create_hull', anonymous=False)
#   publish_marker()

if __name__ == '__main__':
    try:
        CreateMarkers()
    except:
        rospy.loginfo("create_hull node terminated.")