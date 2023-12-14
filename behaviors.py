#!/usr/bin/env python3

import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate(pt.behaviour.Behaviour):
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        super(Rotate, self).__init__(name)
        self.ang_vel = ang_vel
        self.topic_name = topic_name
        self.publisher = None

    def setup(self, **kwargs):
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
            self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
            return True
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

    def update(self):
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        twist = Twist()
        twist.angular.z = self.ang_vel
        self.publisher.publish(twist)

        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        stop_twist = Twist()
        stop_twist.angular.z = 0
        self.publisher.publish(stop_twist)
        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    def __init__(self, name="stop motion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.topic_name = topic_name
        self.publisher = None

    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
            self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
            return True
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

    def update(self):
        self.logger.info("[STOP MOTION] update: updating stop motion behavior")

        stop_twist = Twist()
        stop_twist.linear.x = 0
        stop_twist.angular.z = 0
        self.publisher.publish(stop_twist)

        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info("[STOP MOTION] terminate: stop motion behavior terminated")
        return super().terminate(new_status)

class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, battery_voltage_topic_name="/battery_voltage",
                 name="Battery2BB",
                 threshold=30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=ptr.utilities.qos_profile_unlatched())

        self.threshold = threshold
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)

    def update(self):
        self.logger.info('[BATTERY] update: running battery_status2bb update')

        battery_level = self.blackboard.battery
        if battery_level <= self.threshold:
            self.blackboard.battery_low_warning = True
            return pt.common.Status.SUCCESS
        else:
            self.blackboard.battery_low_warning = False
            return pt.common.Status.RUNNING

class LaserScan2bb(ptr.subscribers.ToBlackboard):
    def __init__(self, topic_name="/scan", name="Scan2BB", safe_range=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan': 'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))

        self.safe_range = safe_range
        self.blackboard.register_key(key='obstacle_nearby', access=pt.common.Access.WRITE)

    def update(self):
        self.logger.info('[LASER SCAN] update: running LaserScan2bb update')

        scan_ranges = self.blackboard.laser_scan
        if any(distance < self.safe_range for distance in scan_ranges if distance is not None):
            self.blackboard.obstacle_nearby = True
            return pt.common.Status.FAILURE
        else:
            self.blackboard.obstacle_nearby = False
            return pt.common.Status.RUNNING
