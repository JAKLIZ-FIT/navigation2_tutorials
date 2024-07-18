import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Quaternion
from nav_msgs.msg import Path
from shapely.geometry import Polygon, Point, LineString
from sensor_msgs.msg import NavSatFix

from my_row_planner import *

class AreaSelector(Node):
    def __init__(self):
        super().__init__('area_selector')
        self.subscription = self.create_subscription(
            PolygonStamped,
            'selected_area',
            self.area_callback,
            10
        )
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)

        self.area = []

        self.field_vertices = []
        self.polygon_vertices_gps = []

        # TODO create 
        self.robot_pos_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10
        )
    
    def area_callback(self, msg):
        self.area = [(point.x, point.y) for point in msg.polygon.points]
        self.polygon_vertices_gps = self.area
        if len(self.area) < 3:
            self.get_logger().info(f'Selected area has only two points: {self.area}')    
            return
        
        self.field_vertices= [Point(point.x, point.y) for point in msg.polygon.points]
        self.get_logger().info(f'Selected area: {self.area}')

        if msg.header.frame_id == 'wgs84':
            print ("points in wgs84 frame, converting to map frame")
            # TODO convert to some local coordinates
        else:
            print (f"points are in {msg.header.frame} frame")

        # TODO add robot position instead of first waypoint
        row_planner = MyRowPlanner(self.polygon_vertices_gps, self.polygon_vertices_gps[0])

        row_path_waypoints = row_planner.generate_row_path()

        path = Path()
        path.header.frame_id = 'wgs84'
        path.header.stamp = self.get_clock().now().to_msg()

        # TODO add yaw computing

        for waypoint in row_path_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'wgs84'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation = Quaternion(w=1.0)  # No rotation
            path.poses.append(pose)

        self.path_publisher_.publish(path)


    
    #     vert_cnt = len(self.field_vertices)
    #     vert_idx = 0
    #     closest_vertex = self.field_vertices[vert_idx] 
    #     smallest_distance = robot_pos.distance(closest_vertex)

    #     for i,field_vertex in enumerate(self.field_vertices):
    #         temp_dist = robot_pos.distance(field_vertex)
    #         if temp_dist < smallest_distance:
    #             closest_vertex = field_vertex
    #             smallest_distance = temp_dist
    #             vert_idx = i

    #     # get neighbors of closest vertex

    #     next_vertex_a = None
    #     next_vertex_b = None

    #     next_vertex_a = self.field_vertices[vert_idx-1] 

    #     if vert_idx == (vert_cnt-1):
    #         next_vertex_b = self.field_vertices[0]    
    #     else:
    #         next_vertex_b = self.field_vertices[vert_idx+1]

    #     # compute lengths of edges connected to closses vertex

    #     edge_a = LineString(robot_pos, next_vertex_a)
    #     edge_b = LineString(robot_pos, next_vertex_b)

    #     chosen_edge = edge_a
    #     chosen_next_vert = next_vertex_a
    #     if edge_a.length < edge_b.length:
    #         chosen_edge = edge_b
    #         chosen_next_vert = next_vertex_b

    #     # compute angle of chosen edge

        
# # compute angle between field edge and East
# def compute_edge_angle(self, edge):
#     edge_coords = edge.coords
#     print(edge_coords)
    
    def gps_callback(self, msg):
        pass # TODO extract gps position of root

def main(args=None):
    rclpy.init(args=args)
    area_selector = AreaSelector()
    rclpy.spin(area_selector)
    area_selector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
