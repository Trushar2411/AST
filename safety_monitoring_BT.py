#!/usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys
from behaviors import *

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behavior tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """
    rotate_behavior = Rotate()
    stop_motion_behavior = StopMotion()
    battery_status_behavior = BatteryStatus2bb()
    laser_scan_behavior = LaserScan2bb()

    root = pt.composites.Parallel("root", policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    topics2BB.add_children([battery_status_behavior, laser_scan_behavior])
    priorities.add_children([rotate_behavior, stop_motion_behavior])

    root.add_children([topics2BB, priorities])
    return root

def main():
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
