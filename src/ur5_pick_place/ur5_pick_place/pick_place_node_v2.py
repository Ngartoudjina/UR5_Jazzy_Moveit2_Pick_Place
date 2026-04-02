#!/usr/bin/env python3
"""UR5 Pick and Place Node v2.0 - Modern refactored version."""

import sys
import rclpy

from ur5_pick_place.base_pick_place import PickPlaceBase
from ur5_pick_place.config_loader import ConfigLoader


class PickPlaceNode(PickPlaceBase):
    """Main pick and place execution node."""

    def __init__(self):
        super().__init__("pick_place_node")

    def run_sequence(self) -> bool:
        """Execute 8-stage pick and place sequence."""

        try:
            # Stage 0: Setup
            stage_0 = self.execute_stage(
                "[0/8] SETUP - Open gripper",
                self._stage_setup
            )
            if not stage_0.success:
                return False

            # Stage 1: Home
            stage_1 = self.execute_stage(
                "[1/8] HOME",
                self._stage_move_home
            )
            if not stage_1.success:
                return False

            # Stage 2: Pre-pick
            stage_2 = self.execute_stage(
                "[2/8] PRE_PICK",
                self._stage_move_pre_pick
            )
            if not stage_2.success:
                return False

            # Stage 3: Grasp
            stage_3 = self.execute_stage(
                "[3/8] GRASP",
                self._stage_move_grasp
            )
            if not stage_3.success:
                return False

            # Stage 4: Close gripper
            stage_4 = self.execute_stage(
                "[4/8] CLOSE",
                self._stage_close_gripper
            )
            if not stage_4.success:
                return False

            # Stage 5: Lift
            stage_5 = self.execute_stage(
                "[5/8] LIFT",
                self._stage_move_lift
            )
            if not stage_5.success:
                return False

            # Stage 6: Pre-place
            stage_6 = self.execute_stage(
                "[6/8] PRE_PLACE",
                self._stage_move_pre_place
            )
            if not stage_6.success:
                return False

            # Stage 7: Place
            stage_7 = self.execute_stage(
                "[7/8] PLACE",
                self._stage_move_place
            )
            if not stage_7.success:
                return False

            # Stage 8: Release
            stage_8 = self.execute_stage(
                "[8/8] RELEASE",
                self._stage_release_object
            )

            return stage_8.success

        except Exception as e:
            self.logger.error(f"Exception in run_sequence: {e}")
            return False

    def _stage_setup(self) -> bool:
        """Setup: open gripper."""
        return self.gripper.async_open()

    def _stage_move_home(self) -> bool:
        """Move to home position."""
        home_pos = ConfigLoader.get_position("home")
        if not home_pos:
            return False
        return self.move_to_position(home_pos)

    def _stage_move_pre_pick(self) -> bool:
        """Move to pre-pick position."""
        pos = ConfigLoader.get_position("pre_pick")
        if not pos:
            return False
        return self.move_to_position(pos)

    def _stage_move_grasp(self) -> bool:
        """Move to grasp position."""
        pos = ConfigLoader.get_position("grasp")
        if not pos:
            return False
        return self.move_to_position(pos)

    def _stage_close_gripper(self) -> bool:
        """Close gripper."""
        profile = ConfigLoader.get_gripper_profile("medium_grasp")
        if not profile:
            return False
        return self.gripper.async_grasp(profile)

    def _stage_move_lift(self) -> bool:
        """Move to lift position."""
        pos = ConfigLoader.get_position("lift")
        if not pos:
            return False
        return self.move_to_position(pos)

    def _stage_move_pre_place(self) -> bool:
        """Move to pre-place position."""
        pos = ConfigLoader.get_position("pre_place")
        if not pos:
            return False
        return self.move_to_position(pos)

    def _stage_move_place(self) -> bool:
        """Move to place position."""
        pos = ConfigLoader.get_position("place")
        if not pos:
            return False
        return self.move_to_position(pos)

    def _stage_release_object(self) -> bool:
        """Open gripper and retreat."""
        if not self.gripper.async_open():
            return False
        
        retreat_pos = ConfigLoader.get_position("retreat")
        if retreat_pos:
            return self.move_to_position(retreat_pos)
        return True


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = PickPlaceNode()
    
    try:
        success = node.run()
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        node.logger.info("Pick and place cancelled")
        exit_code = 1
    except Exception as e:
        node.logger.error(f"Fatal error: {e}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
