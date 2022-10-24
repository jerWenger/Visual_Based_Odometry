import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class VisualizeTagNode(Node):
    def __init__(self):
        super().__init__('visualize_tag_node')

        self.camera_calibration_path = '/home/ali1/ros2_ws/src/Visual_Based_Odometry/Calibration Data/ost.yaml'

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.camera_info_msg)

        self.camera_publisher = self.create_publisher(CameraInfo, 'neato_camera_info', 10)

    def camera_info_msg(self):
        self.camera_publisher.publish(self.yaml_to_CameraInfo(self.camera_calibration_path))

    def yaml_to_CameraInfo(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.
        
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.k = calib_data["camera_matrix"]["data"]
        camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.r = calib_data["rectification_matrix"]["data"]
        camera_info_msg.p = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg
        
def main(args=None):
    rclpy.init(args=args)
    node = VisualizeTagNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()