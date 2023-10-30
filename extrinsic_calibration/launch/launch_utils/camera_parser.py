import yaml
from launch_ros.actions import  Node
from ament_index_python.packages import get_package_share_directory
import os

class Camera:
    model_name = ""
    type = ""
    x=0.0
    y=0.0
    z=0.0
    yaw=0.0
    pitch=0.0
    roll=0.0
    # cam type -> model type
    supported_cams = {"fisheye": "fisheye_camera"}
    
    def get_launch_descriptor(self, use_world_frame = True):
        if(self.type in self.supported_cams.keys()):
            model = self.supported_cams[self.type]
        else:
            raise ValueError("The cameras in the yaml file should be given as a list under the `cameras` key")
        return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', str(self.model_name),
            '-database', str(model),
            '-robot_namespace', str(self.model_name),
            '-reference_frame', "robot::base_link",
            '-x', str(self.x), '-y', str(self.y), '-z', str(self.z),
            '-R', str(self.roll), '-P', str(self.pitch), '-Y', str(self.yaw)])
    

def read_camera_from_yaml(yaml_file: str):
    with open(yaml_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
        cameras = yaml_data["cameras"]
        if isinstance(cameras, list):
            for item in cameras:
                camera = Camera()
                camera.model_name = item["id"]
                camera.type = item["type"]
                camera.x = item["pose"]["x"]
                camera.y = item["pose"]["y"]
                camera.z = item["pose"]["z"]
                camera.yaw = item["pose"]["yaw"]
                camera.pitch = item["pose"]["pitch"]
                camera.roll = item["pose"]["roll"]
                yield camera
        else:
            raise ValueError("The cameras in the yaml file should be given as a list under the `cameras` key")
        
def main():
    config_file = os.path.join(get_package_share_directory("extrinsic_calibration"), "config", "world_setup.yaml")
    print([camera.get_launch_descriptor() for camera in read_camera_from_yaml(config_file)])

if __name__ == "__main__":
    main()
