#!/usr/bin/python

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList, GeoVizPolygon
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint

from robot_localization.srv import *

def toLL(x, y, z=0.0):
    rospy.wait_for_service('/cora/robot_localization/toLL')
    try:
        sp = rospy.ServiceProxy('/cora/robot_localization/toLL', ToLL)
        r = ToLLRequest()
        r.map_point.x = x
        r.map_point.y = y
        r.map_point.z = z
        return sp(r).ll_point
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def costmap_callback(data):
    try:
        transformation = tf_buffer.lookup_transform('map', data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
    except Exception as e:
        rospy.logwarn(str(e))
    else:
        origin = PoseStamped()
        origin.pose = data.info.origin
        
        origin_map = tf2_geometry_msgs.do_transform_pose(origin, transformation)
        origin_ll = toLL(origin_map.pose.position.x,origin_map.pose.position.y,origin_map.pose.position.z)
        
        height_meters = data.info.resolution*data.info.height
        width_meters = data.info.resolution*data.info.width
        opposite_ll = toLL(origin_map.pose.position.x+height_meters,origin_map.pose.position.y+width_meters,origin_map.pose.position.z)
        
        vizItem = GeoVizItem()
        
        vizItem.id = 'occupency_grid'
        plist = GeoVizPointList()
        plist.color.r = 0.0
        plist.color.g = 0.5
        plist.color.b = 1.0
        plist.color.a = 1.0
        corners = [origin_ll,opposite_ll]
        for i in ( (0,0), (0,1), (1,1), (1,0), (0,0) ):
            gp = GeoPoint()
            gp.latitude = corners[i[0]].latitude
            gp.longitude = corners[i[1]].longitude
            plist.points.append(gp)
        vizItem.lines.append(plist)
        
        dlat = (opposite_ll.latitude - origin_ll.latitude)/float(data.info.width)
        dlong = (opposite_ll.longitude - origin_ll.longitude)/float(data.info.height)
        

        for row in range(data.info.height):
            for col in range(data.info.width):
                if data.data[row*data.info.width+col] > 0:
                    p = GeoVizPolygon()
                    p.fill_color.r = 0.1
                    p.fill_color.g = 0.1
                    p.fill_color.b = 0.1
                    p.fill_color.a = 0.7
                    for i in ( (0,0), (0,1), (1,1), (1,0), (0,0) ):
                        gp = GeoPoint()
                        gp.latitude = origin_ll.latitude+dlat*(row+i[0])
                        gp.longitude = origin_ll.longitude+dlong*(col+i[1])
                        p.outer.points.append(gp)

                    vizItem.polygons.append(p)
        display_publisher.publish(vizItem)
        
grid_sub = None
# allow tf_buffer to fill up a bit before asking for a grid
def delayed_subscribe(data):
    global grid_sub
    grid_sub = rospy.Subscriber('occupancy_grid', OccupancyGrid, costmap_callback)

rospy.init_node("occupancy_to_camp")

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size = 10)

rospy.Timer(rospy.Duration(2), delayed_subscribe, oneshot=True)

rospy.spin()
