#! /usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, Image

import argparse
import yaml


class CameraInfoPub:
    def __init__(self, filename="", camera_info={}):
        self.pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
        self.sub = rospy.Subscriber('image', Image, self.image_cb)

        if len(filename) == 0 and len(camera_info) == 0:
            rospy.logwarn('Default camera info will be published')
        if len(filename) and len(camera_info):
            rospy.logwarn('Supplied camera_info takes prioriy over filename')
        info = {'image_height': 0, 'image_width':0,
                'distortion_model': '',
                'distortion_coefficients': {'data': [0, 0, 0, 0, 0]},
                'camera_matrix': {'data': [0, 0, 0, 0, 0, 0, 0, 0, 0]},
                'rectification_matrix': {'data': [0, 0, 0 , 0, 0, 0, 0, 0, 0]},
                'projection_matrix': {'data': [0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0]}}
        if len(filename):
            with open(filename, 'r') as stream:
                info = yaml.safe_load(stream)
        if len(camera_info):
            info = camera_info

        self.info = CameraInfo(height=info['image_height'],
                               width=info['image_width'],
                               distortion_model=info['distortion_model'],
                               D=info['distortion_coefficients']['data'],
                               K=info['camera_matrix']['data'],
                               R=info['rectification_matrix']['data'],
                               P=info['projection_matrix']['data'])
        rospy.loginfo('[{}] Loaded camera_info:\n{}'.format(rospy.get_name(),
                                                            self.info))

    def image_cb(self, data):
        self.info.header = data.header
        self.pub.publish(self.info)


def parse_cli():
    parser = argparse.ArgumentParser(
        description='Publish camera_info in sync with image')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--file', '-f', default="",
                       help='path to YAML file with camera calibration info')
    group.add_argument('--camera_info', '-c', default="",
                       help='camera info in YAML format')
    return parser.parse_known_args()


def main():
    rospy.init_node('camera_info_publisher', anonymous=True)
    config, _ = parse_cli()
    if len(config.camera_info):
        camera_info_pub = CameraInfoPub(yaml.safe_load(config.camera_info))
    else:
        camera_info_pub = CameraInfoPub(config.file)
    rospy.spin()


if __name__ == '__main__':
    main()
