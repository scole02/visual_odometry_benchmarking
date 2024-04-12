#!/usr/bin/env python

"""
TODO
groundtruth_dataset_name.npy # only needs to be saved once
output_dataset_method_timestamp.npy # saved for each run
should contain flattened pose and twist (13 elems?)

Write a ros wrapper for a new method if does not exist.

1. How should odom topics be deserialized into .npy? Just rospy.np
"""
import rospy

NAMESPACE = "experiment_listener"

class ExperimentListener:
    def __init__(self, ns):
        params = rospy.get_param(ns) # get dict of params


def main():
    rospy.init_node('experiment_listener')
    listener = ExperimentListener(NAMESPACE)

if __name__ == '__main__':
    main()
