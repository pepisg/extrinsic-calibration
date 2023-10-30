import yaml
from transforms3d.euler import euler2mat, mat2euler
from numpy import matmul
from launch_ros.actions import  Node
from ament_index_python.packages import get_package_share_directory
import os

class Apriltag:
    model_name = ""
    x=0.0
    y=0.0
    z=0.0
    yaw=0.0
    pitch=0.0
    roll=0.0

    def transform_to_world_frame(self):
        # apriltags z axis its orthogonal to its surface. We want
        # the z axis to be aligned with the world z axis, thus we 
        # increase the pitch by 90 degrees and assume the other angles
        # are given in world coordinates
        apriltag_to_world_rot = euler2mat(0.0, -1.57, 0.0)
        apriltag_config_rot = euler2mat(self.roll, self.pitch, self.yaw)
        full_rot = matmul(apriltag_config_rot, apriltag_to_world_rot)
        self.roll, self.pitch, self.yaw = mat2euler(full_rot)

    
    def get_launch_descriptor(self, use_world_frame = True):
        if(use_world_frame):
            self.transform_to_world_frame()
        return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', str(self.model_name),
            '-database', str(self.model_name) ,
            '-x', str(self.x), '-y', str(self.y), '-z', str(self.z),
            '-R', str(self.roll), '-P', str(self.pitch), '-Y', str(self.yaw)])
    

def read_apriltag_from_yaml(yaml_file: str):
    with open(yaml_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
        apriltags = yaml_data["apriltags"]
        if isinstance(apriltags, list):
            for item in apriltags:
                apriltag = Apriltag()
                apriltag.model_name = item["id"]
                apriltag.x = item["pose"]["x"]
                apriltag.y = item["pose"]["y"]
                apriltag.z = item["pose"]["z"]
                apriltag.yaw = item["pose"]["yaw"]
                apriltag.pitch = item["pose"]["pitch"]
                apriltag.roll = item["pose"]["roll"]
                yield apriltag
        else:
            raise ValueError("The apriltags in the yaml file should be given as a list under the `apriltags` key")
        
def main():
    config_file = os.path.join(get_package_share_directory("extrinsic_calibration"), "config", "world_setup.yaml")
    print([apriltag.get_launch_descriptor() for apriltag in read_apriltag_from_yaml(config_file)])

if __name__ == "__main__":
    main()
