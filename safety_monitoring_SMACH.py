import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import smach
class Monitor(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['lvl_battery', 'collision'])
        self.node = node
    def execute(self, userdata):
        self.node.get_logger().info('In Monitor1235')
        while self.node.current_state == 'Monitor':
            rclpy.spin_once(self.node)
            if self.node.current_state == 'StopBase':
                return 'collision'
            elif self.node.current_state == 'RotateBase':
                return 'lvl_battery'
class RotateBase(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['lvl_battery'])
        self.node = node
    def execute(self, userdata):
        self.node.get_logger().info('In RotateBase')
        while self.node.current_state == 'RotateBase':
            rclpy.spin_once(self.node)
        return 'lvl_battery'
class StopBase(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['collision'])
        self.node = node
    def execute(self, userdata):
        self.node.get_logger().info('In StopBase')
        while self.node.current_state  == 'StopBase':
            rclpy.spin_once(self.node)
        return 'collision'
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        #msg.ranges = None
        #self.scan_ranges = None
        self.battery_subscription = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.battery_callback,
            10)
        #self.subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        #self.subscription
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['outcome4'])
        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('Monitor', Monitor(self), transitions={'lvl_battery':'RotateBase', 'collision':'StopBase'})
            smach.StateMachine.add('RotateBase', RotateBase(self), transitions={'lvl_battery':'Monitor'})
            smach.StateMachine.add('StopBase', StopBase(self), transitions={'collision':'Monitor'})
        self.current_state = 'Monitor'
        #self.aux_state = 'Monitor'
        self.sm.execute()  # Start the state machine
    def battery_callback(self, msg):
        if msg.data < 20 and self.current_state != 'RotateBase':
            self.current_state = 'RotateBase'
        elif msg.data >= 20 and self.current_state != 'Monitor':
            self.current_state = 'Monitor'
    def scan_callback(self, msg):
        if min(msg.ranges) < 30 and self.current_state != 'StopBase':
            self.current_state = 'StopBase'
        elif min(msg.ranges) >= 30 and self.current_state != 'Monitor':
            self.current_state = 'Monitor'
def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)
    state_machine_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()