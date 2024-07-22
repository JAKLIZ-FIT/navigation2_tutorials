import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import yaml
import os
import sys

my_src_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_src_dir)

from my_row_planner import MyCoordTransformer 
import numpy as np
import pandas as pd

class GpsLogger(Node):
    """
    ROS2 node to log GPS position for evaluation of gps accuracy
    """

    def __init__(self, logging_file_path):
        Node.__init__(self, 'gps_logger')
        
        self.logging_file_path = logging_file_path

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'ublox_gps_node/fix',
            self.gps_callback,
            1
        )
        self.last_gps_position = NavSatFix()

        self.gps_pos_list = []
        self.local_pos_list = []
        self.reached_target = False

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg

        if len(self.gps_pos_list) < 600:
            self.gps_pos_list.append({"latitude" :msg.latitude,
                                      "longitude":msg.longitude,
                                      "yaw": 0})  
        else:
            if self.reached_target:
                pass
            else:
                print("reached target number of positions")
                self.reached_target = True
                self.calculate_metrics()
                # self.log_positions()
                

    def calculate_metrics(self):
        coord_transformer = MyCoordTransformer(self.gps_pos_list[0]["latitude"],self.gps_pos_list[0]["longitude"])
        self.local_pos_list = [coord_transformer.gps_to_local(p["latitude"],p["longitude"]) for p in self.gps_pos_list]

        pos_array = np.array(self.local_pos_list)
        p_mean = pos_array.mean(axis=0)
        print(f"average position = {p_mean}")
        p_max = pos_array.max(axis=0)
        p_min = pos_array.min(axis=0)
        print(f"deviation range: x ({p_min[0]- p_mean[0]}; {p_max[0]-p_mean[0]}) y ({p_min[1]-p_mean[1]};{p_max[1]-p_mean[1]})")
        print(f"variance = {pos_array.var(axis=0, ddof=1)}")

        col_names = ['x', 'y']
        df_local = pd.DataFrame(self.local_pos_list, columns=col_names)
        print(df_local.describe())

        df_local.to_csv("gps_pos_log_localCoord.csv")





    def log_positions(self):
        """
        Function to save logged positions to a file
        """
    
        # build new waypoint object
        
        yaml_data = {"waypoints" : self.gps_pos_list}

        # write updated waypoints
        try:
            with open(self.logging_file_path, 'w') as yaml_file:
                yaml.dump(yaml_data, yaml_file, default_flow_style=False)
        except Exception as ex:
            print(f"Error logging position: {str(ex)}")
            return

        

    



def main(args=None):
    rclpy.init(args=args)

    # allow to pass the logging path as an argument
    default_yaml_file_path = os.path.expanduser("~/gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_gui_logger = GpsGuiLogger(yaml_file_path)

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(gps_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        gps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
