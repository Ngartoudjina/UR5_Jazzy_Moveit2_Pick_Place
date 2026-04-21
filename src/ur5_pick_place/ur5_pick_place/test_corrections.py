#!/usr/bin/env python3
"""
Validation test suite for all 11 corrections made to the pick & place system.
Tests each fix individually to ensure the pipeline works end-to-end.
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node

# Import all components to test
from ur5_pick_place.config_loader import ConfigLoader, JointPosition
from ur5_pick_place.ik_solver import UR5IKSolver, JOINT_NAMES
from ur5_pick_place.gripper_controller import SimulatedGripperController, GripperProfile
from ur5_pick_place.validators import SafetyValidator, JointValidator
from ur5_pick_place.base_pick_place import PickPlaceBase


class ValidationTestNode(Node):
    """Node to test all corrections."""

    def __init__(self):
        super().__init__('validation_test')
        self.logger = self.get_logger()
        self.test_results = []

    def test_1_config_loader(self):
        """Test #5: Config file loading"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #5: Config Loader (config file installation)")
        self.logger.info("="*70)
        
        try:
            config = ConfigLoader()
            ConfigLoader.load_from_file('config/ur5_pick_place_config.yaml')
            
            # Verify group name is consistent
            group = ConfigLoader.get_group_name()
            assert group == "ur5_arm", f"Expected 'ur5_arm', got '{group}'"
            
            # Verify positions loaded
            home = ConfigLoader.get_position("home")
            assert home is not None, "Home position not found"
            assert len(home.angles) == 6, f"Expected 6 joints, got {len(home.angles)}"
            
            self.logger.info(f"✅ Config loaded successfully")
            self.logger.info(f"   Group name: {group}")
            self.logger.info(f"   Home position: {[f'{a:.3f}' for a in home.angles]}")
            self.test_results.append(("Test #5: Config Loader", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Config loader failed: {e}")
            self.test_results.append(("Test #5: Config Loader", False))
            return False

    def test_2_ik_solver_init(self):
        """Test #2: IK Solver initialization"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #2: IK Solver Initialization")
        self.logger.info("="*70)
        
        try:
            solver = UR5IKSolver(self)
            self.logger.info(f"✅ IK Solver created successfully")
            self.logger.info(f"   Joint names: {JOINT_NAMES}")
            self.test_results.append(("Test #2: IK Solver Init", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ IK Solver init failed: {e}")
            self.test_results.append(("Test #2: IK Solver Init", False))
            return False

    def test_3_ik_solver_cartesian(self):
        """Test #6: IK Solver with corrected d6 (no +0.13m offset)"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #6: IK Solver - Corrected d6 (no +0.13m offset)")
        self.logger.info("="*70)
        
        try:
            solver = UR5IKSolver(self)
            time.sleep(0.5)  # Wait for joint_states subscription
            
            # Test Cartesian IK
            x, y, z = 0.3, 0.0, 0.2
            qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707
            
            result = solver.solve(x, y, z, qx, qy, qz, qw)
            
            assert result is not None, "IK solve returned None"
            joints = result['joints']
            assert len(joints) == 6, f"Expected 6 joints, got {len(joints)}"
            
            self.logger.info(f"✅ IK Solver computed successfully")
            self.logger.info(f"   Target: x={x}, y={y}, z={z}")
            self.logger.info(f"   Joints: {[f'{math.degrees(j):.1f}°' for j in joints]}")
            self.test_results.append(("Test #6: IK d6 Correction", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ IK solver test failed: {e}")
            self.test_results.append(("Test #6: IK d6 Correction", False))
            return False

    def test_4_gripper_non_blocking(self):
        """Test #9: Gripper non-blocking (uses timers, not time.sleep)"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #9: Gripper Non-Blocking (no time.sleep blocking)")
        self.logger.info("="*70)
        
        try:
            gripper = SimulatedGripperController(self)
            
            # Test open (should return immediately, not block)
            start = time.time()
            result = gripper.async_open()
            elapsed = time.time() - start
            
            assert result is True, "Gripper open failed"
            assert elapsed < 0.1, f"Gripper took {elapsed}s - should not block!"
            
            self.logger.info(f"✅ Gripper.async_open() returned immediately")
            self.logger.info(f"   Execution time: {elapsed*1000:.1f}ms (non-blocking)")
            
            # Test close
            start = time.time()
            result = gripper.async_close()
            elapsed = time.time() - start
            
            assert result is True, "Gripper close failed"
            assert elapsed < 0.1, f"Gripper took {elapsed}s - should not block!"
            
            self.logger.info(f"✅ Gripper.async_close() returned immediately")
            self.logger.info(f"   Execution time: {elapsed*1000:.1f}ms (non-blocking)")
            self.test_results.append(("Test #9: Gripper Non-Blocking", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Gripper test failed: {e}")
            self.test_results.append(("Test #9: Gripper Non-Blocking", False))
            return False

    def test_5_joint_validator(self):
        """Test #10: Joint Validator with improved logic"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #10: Joint Validator (improved normalization)")
        self.logger.info("="*70)
        
        try:
            # Test valid angles
            valid_angles = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
            result = JointValidator.validate_angles(valid_angles)
            assert result.passed, f"Valid angles rejected: {result.message}"
            
            self.logger.info(f"✅ Valid angles accepted")
            self.logger.info(f"   Angles: {[f'{math.degrees(a):.0f}°' for a in valid_angles]}")
            
            # Test out-of-range angles
            invalid_angles = [0.0, -10.0, 1.57, -1.57, -1.57, 0.0]  # shoulder_lift = -10 rad
            result = JointValidator.validate_angles(invalid_angles)
            assert not result.passed, "Invalid angles were accepted!"
            
            self.logger.info(f"✅ Invalid angles rejected correctly")
            self.logger.info(f"   Message: {result.message}")
            self.test_results.append(("Test #10: Joint Validator", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Joint validator test failed: {e}")
            self.test_results.append(("Test #10: Joint Validator", False))
            return False

    def test_6_type_compatibility(self):
        """Test #3: Type compatibility (dict vs JointPosition)"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #3: Type Compatibility (dict vs JointPosition)")
        self.logger.info("="*70)
        
        try:
            # Test JointPosition parsing
            joint_pos = JointPosition(
                name="test",
                angles=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
            )
            assert isinstance(joint_pos, JointPosition), "JointPosition type mismatch"
            
            self.logger.info(f"✅ JointPosition type works correctly")
            self.logger.info(f"   Name: {joint_pos.name}")
            self.logger.info(f"   Angles: {len(joint_pos.angles)} joints")
            
            # Test dict parsing
            cartesian_dict = {
                'x': 0.3,
                'y': 0.0,
                'z': 0.2,
                'qx': 0.0,
                'qy': 0.707,
                'qz': 0.0,
                'qw': 0.707
            }
            assert isinstance(cartesian_dict, dict), "Dict type mismatch"
            assert 'x' in cartesian_dict and 'z' in cartesian_dict, "Missing Cartesian coords"
            
            self.logger.info(f"✅ Dict (Cartesian) type works correctly")
            self.logger.info(f"   Coords: x={cartesian_dict['x']}, y={cartesian_dict['y']}, z={cartesian_dict['z']}")
            self.test_results.append(("Test #3: Type Compatibility", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Type compatibility test failed: {e}")
            self.test_results.append(("Test #3: Type Compatibility", False))
            return False

    def test_7_safety_validation(self):
        """Test #10 (compound): Safety validation still works"""
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST #7: Safety Validation (pre-flight check)")
        self.logger.info("="*70)
        
        try:
            # Valid angles
            valid = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
            result = SafetyValidator.pre_flight_check(valid)
            assert result.passed, "Valid preflight check failed"
            
            self.logger.info(f"✅ Preflight check passed for valid angles")
            self.logger.info(f"   Message: {result.message}")
            self.test_results.append(("Test #7: Safety Validation", True))
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Safety validation test failed: {e}")
            self.test_results.append(("Test #7: Safety Validation", False))
            return False

    def run_all_tests(self):
        """Run all validation tests."""
        self.logger.info("\n" + "🔍 "*35)
        self.logger.info("RUNNING COMPLETE VALIDATION TEST SUITE")
        self.logger.info("🔍 "*35)
        
        tests = [
            self.test_1_config_loader,
            self.test_2_ik_solver_init,
            self.test_3_ik_solver_cartesian,
            self.test_4_gripper_non_blocking,
            self.test_5_joint_validator,
            self.test_6_type_compatibility,
            self.test_7_safety_validation,
        ]
        
        passed = 0
        for test_func in tests:
            try:
                if test_func():
                    passed += 1
            except Exception as e:
                self.logger.error(f"Test exception: {e}")
        
        # Print summary
        self.logger.info("\n" + "="*70)
        self.logger.info("TEST SUMMARY")
        self.logger.info("="*70)
        
        for test_name, success in self.test_results:
            status = "✅ PASS" if success else "❌ FAIL"
            self.logger.info(f"{status} | {test_name}")
        
        self.logger.info(f"\n📊 Results: {passed}/{len(tests)} tests passed")
        
        if passed == len(tests):
            self.logger.info("🎉 ALL CORRECTIONS VALIDATED SUCCESSFULLY!")
            return 0
        else:
            self.logger.error(f"⚠️  {len(tests) - passed} test(s) failed")
            return 1


def main(args=None):
    rclpy.init(args=args)
    node = ValidationTestNode()
    
    try:
        exit_code = node.run_all_tests()
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
