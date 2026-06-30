#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import the RWS service definitions
from abb_robot_msgs.srv import SetRAPIDSymbol, SetRAPIDBool

class RWSCommander(Node):
    def __init__(self):
        super().__init__('rws_commander_node')
        
        # 1. Create service clients connecting to the abb_rws_client node
        self.cli_symbol = self.create_client(SetRAPIDSymbol, '/rws_client/set_rapid_symbol')
        self.cli_bool = self.create_client(SetRAPIDBool, '/rws_client/set_rapid_bool')
        
        # Wait for the abb_rws_client to be online
        while not self.cli_symbol.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /rws_client/set_rapid_symbol service...')
            
    def move_to_joints(self, j1, j2, j3, j6):
        """Sends a 4-axis palletizer coordinate to the ABB controller and executes it."""
        
        # STEP 1: Format the coordinate string for RAPID
        # Notice how user inputs j1/j2/j3/j6 map precisely to the 1st/2nd/3rd/6th array slots!
        target_str = f"[[{j1:.2f}, {j2:.2f}, {j3:.2f}, 0.0, 0.0, {j6:.2f}], [9E9,9E9,9E9,9E9,9E9,9E9]]"
        
        # Prepare the SetRAPIDSymbol request
        req_sym = SetRAPIDSymbol.Request()
        req_sym.path.task = "T_ROB1"
        req_sym.path.module = "Hybrid_Control"
        req_sym.path.symbol = "ros_target"
        req_sym.value = target_str
        
        self.get_logger().info(f"Writing target: {target_str}")
        future_sym = self.cli_symbol.call_async(req_sym)
        rclpy.spin_until_future_complete(self, future_sym) # Block until it succeeds
            
        # STEP 2: Trigger the execute boolean
        req_bool = SetRAPIDBool.Request()
        req_bool.path.task = "T_ROB1"
        req_bool.path.module = "Hybrid_Control"
        req_bool.path.symbol = "execute_ros_move"
        req_bool.value = True
        
        self.get_logger().info("Triggering execution flag...")
        future_bool = self.cli_bool.call_async(req_bool)
        rclpy.spin_until_future_complete(self, future_bool) # Block until it succeeds
        
        self.get_logger().info("Robot is now moving!")

def main():
    rclpy.init()
    node = RWSCommander()
    
    # ── Example Palletizing Sequence ──
    
    # 1. Move to a "Pick" position over a box
    node.get_logger().info("--- Going to Pick Position ---")
    node.move_to_joints(0.0, 30.0, 45.0, 90.0)
    
    # In a real application, you would now use a GetRAPIDSymbol service call in a while loop
    # to poll a "ros_status" variable until it says "DONE" before moving on.
    import time
    time.sleep(3.0) # Fake delay representing the robot moving
    
    # 2. Move to a "Drop" position over the pallet
    node.get_logger().info("--- Going to Drop Position ---")
    node.move_to_joints(45.0, 15.0, 10.0, 0.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
