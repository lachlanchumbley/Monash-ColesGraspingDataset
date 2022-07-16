import os
import glob
import shutil
import json

import cv2
import open3d as o3d

from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as dict2rosmsg

### PRESS [ESC] IN FIGURES TO VISUALIZE NEXT GRASP 

FILE_DIR = './'
DATA_DIR = './'

def main():
    json_dirs = sorted(glob.glob(FILE_DIR + DATA_DIR + 'json_files/*/'))
    rgb_dirs = sorted(glob.glob(FILE_DIR + DATA_DIR + 'rgb_images/*/'))
    depth_dirs = sorted(glob.glob(FILE_DIR + DATA_DIR + 'depth_images/*/'))
    pcl_dirs = sorted(glob.glob(FILE_DIR + DATA_DIR + 'point_clouds/*/'))

    for json_f, rgb_f, depth_f, pcl_f in zip(json_dirs, rgb_dirs, depth_dirs, pcl_dirs):
        json_dirs_2 = sorted(os.listdir(json_f))
        rgb_dirs_2 = sorted(os.listdir(rgb_f))
        depth_dirs_2 = sorted(os.listdir(depth_f))
        pcl_dirs_2 = sorted(os.listdir(pcl_f))
        for json_dir, rgb, depth, pcl in zip(json_dirs_2, rgb_dirs_2, depth_dirs_2, pcl_dirs_2):

            with open(json_f+json_dir, 'r') as json_file:
                json_dict = json.load(json_file)

                object_id = json_dict['object_id']
                translation = json_dict['translation']
                rotation = json_dict['rotation']
                cam_info = dict2rosmsg('sensor_msgs/CameraInfo', json_dict['camera_info'])
                grasp_pose = dict2rosmsg('geometry_msgs/PoseStamped', json_dict['grasp_pose'])
                robot_pose = dict2rosmsg('geometry_msgs/PoseStamped', json_dict['robot_pose'])
                success = json_dict['success']
                stable_success = json_dict['stable_success']

                print('\nObject ID:',object_id)
                print('\nTranslation:',translation)
                print('\nRotation:',rotation)
                print('\nCamera Info:\n',cam_info)
                print('\nGrasp Pose:\n',grasp_pose)
                print('\nRobot Pose:\n',robot_pose)
                print('\nSuccessful Grasp:',success)
                print('\nStable Grasp:',stable_success)

            rgb_img = cv2.imread(rgb_f+rgb)
            depth_img = cv2.imread(depth_f+depth)
            pcl_obj = o3d.io.read_point_cloud(pcl_f+pcl)

            o3d.visualization.draw_geometries([pcl_obj])

            cv2.imshow("rgb", rgb_img)
            cv2.imshow("depth", depth_img)
        
            cv2.waitKey(0)
            

if __name__ == "__main__":

    if not os.path.exists(FILE_DIR + DATA_DIR + 'point_clouds/'):
        os.makedirs(FILE_DIR + DATA_DIR + 'point_clouds/')

    for pcl_folder in glob.glob(FILE_DIR + DATA_DIR + 'point_clouds_*/'):
        for sf in glob.glob(pcl_folder + '*/'):
            shutil.move(sf, FILE_DIR + DATA_DIR + 'point_clouds/' + sf.split('/')[-2])
        os.rmdir(pcl_folder)

    main()