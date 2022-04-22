#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import time

class ResizedClient():
    def __init__(self, node):
        self.client = dynamic_reconfigure.client.Client(node)

    def update(self):
        # params = {'footprint': '[[-0.34,-0.35],[-0.34,0.35],[-0.4, 0.4], [-0.4, 0.6], [0, 0.6], [0, 0.4],[0.34,0.35],[0.4,0],[0.34,-0.35]]'}
        params = {'footprint': '[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]'}
        config = self.client.update_configuration(params)


if __name__ == '__main__':
    rospy.init_node('resized_footprint')
    client = ResizedClient('/move_base_node/local_costmap')
    client.update()
