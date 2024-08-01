from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
import os
import sys
import yaml

from pyproj import Proj, transform

class MySimpleWaypointGPS():
    def __init__(self, lat, lon, yaw) -> None:
        self.latitude = lat
        self.longtitude = lon
        self.yaw = yaw

class MySimpleWaypointLocal():
    def __init__(self, x, y, yaw) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw


class FieldEdge():
    def __init__(self, vertex1:Point, vertex2:Point) -> None:
        self.vertex1 = vertex1
        self.vertex2 = vertex2
        self.length = vertex1.distance(vertex2)
        self.k = (vertex2.y - vertex1.y) / (vertex2.x - vertex1.x)
        # TODO fix division by zero
        self.angle_rad = math.atan(self.k)
        self.q = vertex1.y - self.k*vertex1.x
        self.intersectPoints = []
        self.lineString = LineString([vertex1,vertex2])

    # compute angle between field edge and East
    def compute_edge_angle(self, point_a, point_b):
        angle_tg = (point_b.y - point_a.y) / (point_b.x - point_a.x)
        return math.atan(angle_tg)
    
    


class MyLine():
    def __init__(self,k,q) -> None:
        self.k = k
        self.q = q
        self.angle_rad = math.atan(self.k) # in range 
        self.id = 0
        self.intersectPoints = []
        self.heading = 0 # this is for oriented row_path, in range 0,2*Pi
        # will be initialized when creating final path

    def intersection(self, edge: FieldEdge = None, k2=0, q2=0):
        if edge != None:
            k2 = edge.k
            q2 = edge.q

        #print(self.k)
        #print(self.q)
        #print(k2)
        #print(q2)

        y = self.k*( (q2 - self.q) / (self.k - k2) )+ self.q
        x = self.get_x(y)
        return x,y
            
    def get_x(self,y):
        return (y-self.q) / self.k

    def get_y(self,x):
        return self.k*x + self.q
    
    def compute_heading(self,direction) -> None:
        p1 = self.intersectPoints[0]
        p2 = self.intersectPoints[1]

        if direction == -1:
            p1,p2 = p2,p1

        heading = 0.0

        if p1.x < p2.x:
            heading = self.angle_rad
        else:
            heading = self.angle_rad + math.pi

        if heading > 2*math.pi:
            heading -= 2*math.pi
        if heading < 0.0:
            heading += 2*math.pi

        self.heading = heading

    
    def generate_intermediate_waypoints(self, direction, waypoint_spacing):
        # TODO this will have to be modified for more complex shapes 
        # or just have lines with 2 intersection points
        # and for other edges have a separate line in edge dictionary 
        w1, w2 = self.intersectPoints[0], self.intersectPoints[1]
        ab_line = LineString( [w1,w2] ) if direction == 1 else LineString([w2,w1])

        waypoint_cnt = (int) (ab_line.length / waypoint_spacing)
        waypoints = []
        for i in range(waypoint_cnt):
            wpi = ab_line.interpolate( (i+1) * waypoint_spacing)
            waypoints.append( MySimpleWaypointLocal(wpi.x,wpi.y,self.heading) )

        return waypoints
    
    def to_waypoints(self, direction, generate_intermediate : bool, waypoint_spacing):
        if direction not in [-1,1]:
            return []
        
        self.compute_heading(direction)
        
        first_wp = self.intersectPoints[1] if direction==-1 else self.intersectPoints[0]
        last_wp =  self.intersectPoints[0] if direction==-1 else self.intersectPoints[1]
        
        wps = [MySimpleWaypointLocal(first_wp.x,first_wp.y,self.heading)]
        
        if generate_intermediate:
            wps.extend(self.generate_intermediate_waypoints(direction,waypoint_spacing))
        
        wps.append(MySimpleWaypointLocal(last_wp.x,last_wp.y,self.heading))
        return wps    



def get_paralel_line(line : MyLine, offset):
    row_width_in_y = offset / math.cos(line.angle_rad)
    return MyLine(line.k,line.q+row_width_in_y)

# TODO add robot turning radius and other params
# but not needed for my differential drive robot
class MyRowPlanner():
    def __init__(self, waypoints, robot_gps=None, row_width=1, waypoint_spacing=1) -> None:
        self.coordTransform = MyCoordTransformer(waypoints[0][0], waypoints[0][1])
        self.area = [self.coordTransform.gps_to_local(w[0],w[1]) for w in waypoints]
        print(self.area)
        #self.area = [(0,0),(23.6,8.55),(23.5,16),(17,19),(5.13,14.1)]
        self.field_vertices = [Point(p) for p in self.area]
        self.robot_pos = Point(-1,-1)
        self.edge_list = []
        self.base_edge = None
        self.row_width = row_width
        self.row_width_in_y = self.row_width
        self.row_dict = {}

        self.waypoint_spacing = waypoint_spacing

        # find max in x, y
        x_coords = [p.x for p in self.field_vertices]
        y_coords = [p.y for p in self.field_vertices]
        self.min_x = min(x_coords)
        self.max_x = max(x_coords)
        self.min_y = min(y_coords)
        self.max_y = max(y_coords)

    def generate_row_path(self, generate_intermediate_wps=True):
        pass
        # compute robot position??
        # lets assume we have coordinates in map frame

        robot_pos = self.robot_pos

        # find the clossest vertex, work will start there

        vert_cnt = len(self.field_vertices)
        vert_idx = 0
        start_vertex = self.field_vertices[vert_idx] 
        smallest_distance = robot_pos.distance(start_vertex)

        for i,field_vertex in enumerate(self.field_vertices):
            temp_dist = robot_pos.distance(field_vertex)
            if temp_dist < smallest_distance:
                start_vertex = field_vertex
                smallest_distance = temp_dist
                vert_idx = i

        print(f"closest vertex: {start_vertex}")

        # get neighbors of start vertex

        nva_idx = vert_idx-1
        nvb_idx = 0 if vert_idx == (vert_cnt-1) else vert_idx+1

        next_vertex_a = self.field_vertices[nva_idx] 
        next_vertex_b = self.field_vertices[nvb_idx]

        print(f"next vertex a: {next_vertex_a}")
        print(f"next vertex b: {next_vertex_b}")
        
        # compute lengths of edges connected to start vertex

        len_a = start_vertex.distance(next_vertex_a)
        len_b = start_vertex.distance(next_vertex_a)

        next_vertex = None
        next_idx = 0
        if len_a > len_b:
            next_vertex = next_vertex_a
            next_idx = nva_idx
        else:
            next_vertex = next_vertex_b
            next_idx = nvb_idx

        print(f"chosen next vertex: {next_vertex}")

        # create edge list
        low_idx = min(vert_idx, next_idx)
        high_idx = max(vert_idx, next_idx)

        if high_idx == low_idx + 1: 
            self.field_vertices = self.field_vertices[high_idx:vert_cnt] + self.field_vertices[0:high_idx]
            
        print ("ordered point list")
        for v in self.field_vertices:
            print(f"{v.x} {v.y},",end="")
            print("")

        for i in range(len(self.field_vertices)-1):
            self.edge_list.append(FieldEdge(self.field_vertices[i],self.field_vertices[i+1]))

        for e in self.edge_list:
            print (f"edge: {e.vertex1.x},{e.vertex1.y} - {e.vertex2.x},{e.vertex2.y}")

        self.base_edge = FieldEdge(self.field_vertices[-1], self.field_vertices[0])
        self.row_width_in_y = self.row_width / math.cos(self.base_edge.angle_rad)

        print(f"base edge angle (rad) = {self.base_edge.angle_rad}")
        print(f"parallel line dist in y: {self.row_width_in_y}")

        # create intersection points

        row_lines = []
        direction = 1
        row_num = direction

        base_row_line = MyLine(self.base_edge.k,self.base_edge.q)
        base_row_line.intersectPoints = [self.base_edge.vertex2, self.base_edge.vertex1] # second point first to match order of other lines
        self.row_dict[0] = base_row_line

        for i,edge in enumerate(self.edge_list):
            print(f"\nProcessing edge {i} ({edge.vertex1.x},{edge.vertex1.y})-({edge.vertex2.x},{edge.vertex2.y})\n")
            # generate parallel linegenerate_intermediate
            path_line = get_paralel_line(self.base_edge, row_num*self.row_width)
            print(f"paralel {row_num}: k={path_line.k} q={path_line.q} angle={path_line.angle_rad}")
            # test up/down

            int_point = self.get_path_edge_intersection(path_line, edge)
                
            if int_point == None:
                print(f"no intersection in direction {direction}")
                direction *= -1
                row_num += direction
                if row_num == 0:
                    row_num = direction
                path_line = get_paralel_line(self.base_edge, row_num*self.row_width)
                print(f"paralel {row_num}: k={path_line.k} q={path_line.q} angle={path_line.angle_rad}")
            
                int_point = self.get_path_edge_intersection(path_line, edge)

            if int_point == None:
                print(f"no intersection in direction {direction}")
                continue

            self.add_row_line_with_intersect(path_line, row_num, int_point)
            print(int_point)

            edge.intersectPoints.append(int_point)

            row_num += direction
            if row_num == 0:
                row_num = direction

            while int_point != None:
                path_line = get_paralel_line(self.base_edge, row_num*self.row_width)
                print(f"paralel {row_num}: k={path_line.k} q={path_line.q} angle={path_line.angle_rad}")
                int_point = self.get_path_edge_intersection(path_line, edge)
                print(int_point)

                if int_point == None:
                    break

                self.add_row_line_with_intersect(path_line, row_num, int_point)
                edge.intersectPoints.append(int_point)

                row_num += direction
                if row_num == 0:
                    row_num = direction

        waypoint_list_local = [] # in local coordinates
        direction = -1

        for k, v in self.row_dict.items():
            print(f"row line {k}: {v.id}, {v.intersectPoints[0]}, {v.intersectPoints[1]}")
        
            waypoint_list_local.extend(v.to_waypoints(direction=direction,
                                                      generate_intermediate=generate_intermediate_wps,
                                                      waypoint_spacing=self.waypoint_spacing))
            direction *= -1

            # if direction == -1:
            #     direction *= -1
            #     waypoint_list_local.append(MySimpleWaypointLocal( v.intersectPoints[1].x,v.intersectPoints[1].y,0) )
            #     waypoint_list_local.append(MySimpleWaypointLocal( v.intersectPoints[0].x,v.intersectPoints[0].y,0) )
            # elif direction == 1:
            #     direction *= -1
            #     waypoint_list_local.append(MySimpleWaypointLocal( v.intersectPoints[0].x,v.intersectPoints[0].y,0) )
            #     waypoint_list_local.append(MySimpleWaypointLocal( v.intersectPoints[1].x,v.intersectPoints[1].y,0) )
            
        #waypoint_list_local = [(w.x,w.y) for w in waypoint_list_local]
        print(waypoint_list_local)
        
        waypoint_list_gps = [self.coordTransform.local_to_gps(w.x, w.y, w.yaw) for w in waypoint_list_local]

        print(waypoint_list_gps)

        return waypoint_list_gps


    def log_waypoint_path(self, waypoints: list, logging_file_path):
        """
        Function to save a waypoint path to a file
        """
        wps = {"waypoints" : [{"latitude": wp["lat"],"longitude": wp["lon"], "yaw": wp["yaw"]} for wp in waypoints] }
        try:
            with open(logging_file_path, 'w+') as yaml_file:
                yaml.dump(wps, yaml_file, default_flow_style=False)
        except Exception as ex:
            print("Error", f"Error logging position: {str(ex)}")
            return

        print("Info", "Waypoint path logged succesfully")



    def add_row_line_with_intersect(self, row_line : MyLine, row_num, int_point):
        if row_num in self.row_dict.keys():
            print(f"row line {row_num} already saved, adding int_point")
            self.row_dict[row_num].intersectPoints.append(int_point)
        else:
            print(f"adding line {row_num} with int point")
            row_line.id = row_num
            row_line.intersectPoints = [int_point]
            self.row_dict[row_num] = row_line



    def get_path_edge_intersection(self, path_line : MyLine, edge : FieldEdge):
        intersection_point = path_line.intersection(edge)

        #print(intersection_point)

        y1 = path_line.get_y(self.min_x)
        y2 = path_line.get_y(self.max_x)

        auxline = LineString([(self.min_x,y1),(self.max_x,y2)])
        if auxline.intersects(edge.lineString):
            return auxline.intersection(edge.lineString)
        else:
            print(intersection_point)
            return None
            
    

class MyCoordTransformer():
    def __init__(self, ref_lat, ref_lon):
        # reference GPS position (latitude, longitude)
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        #print(self.ref_lat)
        #print(self.ref_lon)
        #self.ref_lat = 49.601441844116536
        #self.ref_lon = 15.940808329700712

        #zone = int((ref_lon + 180) // 6) + 1
        zone = 33
        print(f"utm_zone = {zone}")
        # Define the local coordinate system using a UTM projection centered around the reference point
        self.proj_utm = Proj(proj='utm', zone=zone, ellps='WGS84', datum='WGS84')
        self.proj_latlon = Proj(proj='latlong', datum='WGS84')

        # reference GPS position in UTM coordinates
        self.ref_easting, self.ref_northing = self.proj_utm(self.ref_lon, self.ref_lat)

    # added with help of chatGPT, TODO transform seems to be deprecated
    def local_to_gps(self,local_x, local_y, heading=0):
        # Convert local coordinates to UTM coordinates
        easting = self.ref_easting + local_x
        northing = self.ref_northing + local_y
        
        # Convert UTM coordinates back to latitude and longitude
        lat, lon = transform(self.proj_utm, self.proj_latlon, easting, northing)
        return {"lat":lat, "lon":lon, "yaw":heading}

    # modified code from chatGPT
    def gps_to_local(self,lat, lon):
        print(f"ref_easting {self.ref_easting}")
        print(f"ref_northing {self.ref_northing}")

        # Convert latitude and longitude to UTM coordinates
        easting, northing = transform(self.proj_latlon, self.proj_utm, lon, lat)
        
        # Convert UTM coordinates to local coordinates
        local_x = easting - self.ref_easting
        local_y = northing - self.ref_northing
        return local_x, local_y

        

def main(args=None):
    
    local_waypoints = [
        (0,0),
        (23.6,8.55),
        (23.5,16),
        (17,19),
        (5.13,14.1)]
    # local_waypoints = [
    #     (0, 0),
    #     (100, 0),
    #     (100, 100),
    #     (0, 100)
    # ]

    # GPS waypoints:
    gps_waypoints = [
        (49.60144184411653, 15.940808329700712),
        (49.601516083397094, 15.941136374073281),
        (49.60158309592855, 15.941136280011598),
        (49.601610807601766, 15.941046855492976),
        (49.6015680749143, 15.940881756139369)
    ]

    demo_gps_waypoints = [
        (49.6014738,15.9407508),
        (49.6014249,15.9407551),
        (49.6014204,15.9406917),
        (49.6014779,15.9406813)
    ]

    # TODO remove 
    gps_waypoints = demo_gps_waypoints

    gps_robot = gps_waypoints[0]

    areaSel = MyRowPlanner(gps_waypoints, gps_robot)
    waypoint_list_gps  = areaSel.generate_row_path(generate_intermediate_wps=True) 

    default_yaml_file_path = os.path.expanduser("~/gps_row_path_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    areaSel.log_waypoint_path(waypoint_list_gps, yaml_file_path)

if __name__ == '__main__':
    main()