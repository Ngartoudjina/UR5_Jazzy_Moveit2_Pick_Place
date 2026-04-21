#!/usr/bin/env python3
"""Pose Sender Node v2.0 - Move to specified position."""

import sys
from typing import List, Optional
import rclpy

from ur5_pick_place.base_pick_place import PickPlaceBase
from ur5_pick_place.config_loader import ConfigLoader, JointPosition


class PoseSenderNode(PickPlaceBase):
    """Move robot to specified position."""

    def __init__(self, position_name: Optional[str] = None, 
                 joint_angles: Optional[List[float]] = None):
        super().__init__("pose_sender_node")
        self.position_name = position_name
        self.joint_angles = joint_angles

    def run_sequence(self) -> bool:
        """Execute movement to target position."""
        try:
            if self.joint_angles:
                pos = JointPosition(
                    name="custom_position",
                    angles=self.joint_angles,
                    description="Custom joint angles"
                )
                return self.move_to_position(pos)
            
            elif self.position_name:
                pos = ConfigLoader.get_position(self.position_name)
                if not pos:
                    self.logger.error(f"Position '{self.position_name}' not found")
                    return False
                
                return self.move_to_position(pos)
            
            else:
                self.logger.error("No position or joint angles provided")
                return False

        except Exception as e:
            self.logger.error(f"Exception: {e}")
            return False


def main(args=None):
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Move UR5 to position")
    parser.add_argument('--position', type=str, help='Named position')
    parser.add_argument('--joints', type=float, nargs=6, help='Joint angles')
    
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    
    node = PoseSenderNode(
        position_name=parsed_args.position,
        joint_angles=parsed_args.joints
    )
    
    try:
        success = node.run()
        exit_code = 0 if success else 1
    except Exception as e:
        node.logger.error(f"Fatal error: {e}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
