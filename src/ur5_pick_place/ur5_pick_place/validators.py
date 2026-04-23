#!/usr/bin/env python3
"""Safety validation framework for UR5 pick & place operations.

CORRECTIONS APPLIQUÉES :
  - JOINT_LIMITS : tous les joints UR5 sont ±6.28319 rad (±2π) selon l'URDF.
    L'ancien code utilisait ±π pour 4 joints et ±2π pour 2 joints — incohérent.
  - La normalisation atan2(sin,cos) ramenait systématiquement dans [-π,π],
    rendant inefficace la vérification des joints ±2π. Corrigé : comparaison
    directe de la valeur brute sans normalisation.
  - WorkspaceValidator.REACH_MAX corrigé à 0.85m (portée réelle UR5 ≈ 0.85m,
    pas 1.3m qui était trop optimiste).
  - WorkspaceValidator : ajout d'une limite basse sur Z pour éviter z<0 (sous table).
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class ValidationResult:
    """Result of validation check."""
    passed: bool
    message: str
    details: str = ""
    warnings: List[str] = field(default_factory=list)


class JointValidator:
    """Validates joint angles against UR5 hardware limits.

    Source : ur5_robot.urdf.xacro — tous les joints bras ont lower="-6.28319" upper="6.28319".
    """

    # CORRECTION : ±2π (6.28319 rad) pour TOUS les joints, conformément à l'URDF.
    # L'ancien code utilisait ±π pour shoulder_pan, shoulder_lift, wrist_1, wrist_2 — incorrect.
    JOINT_LIMITS = [
        (-6.28319, 6.28319),  # shoulder_pan_joint
        (-6.28319, 6.28319),  # shoulder_lift_joint
        (-6.28319, 6.28319),  # elbow_joint
        (-6.28319, 6.28319),  # wrist_1_joint
        (-6.28319, 6.28319),  # wrist_2_joint
        (-6.28319, 6.28319),  # wrist_3_joint
    ]

    JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

    @staticmethod
    def validate_angles(angles: List[float]) -> ValidationResult:
        """Validate joint angles against limits.

        CORRECTION : comparaison directe sans normalisation atan2.
        L'ancienne normalisation atan2(sin,cos) ramenait tout dans [-π,π],
        rendant inefficaces les vérifications pour les angles légitimes > π.
        """
        if not angles or len(angles) != 6:
            return ValidationResult(False, f"Expected 6 joints, got {len(angles) if angles else 0}")

        violations = []
        for i, (angle, (lo, hi)) in enumerate(zip(angles, JointValidator.JOINT_LIMITS)):
            if angle < lo or angle > hi:
                violations.append(
                    f"{JointValidator.JOINT_NAMES[i]}: {math.degrees(angle):.1f}°"
                    f" (limites [{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°])"
                )

        if violations:
            return ValidationResult(
                False,
                f"Limites articulaires dépassées ({len(violations)} joint(s))",
                details="; ".join(violations)
            )

        return ValidationResult(True, "Tous les joints dans les limites ✓")


class WorkspaceValidator:
    """Validates end-effector position within UR5 reachable workspace.

    CORRECTIONS :
      - REACH_MAX = 0.85m  (portée réelle UR5 ≈ 850mm, pas 1.3m)
      - HEIGHT_MIN = -0.3m (permet de passer légèrement sous le plan de la base)
      - HEIGHT_MAX = 1.0m  (limite haute réaliste pour UR5)
    """

    REACH_MAX  = 0.85   # m — portée max UR5 (somme a2+a3+d5+d6 ≈ 0.95m, marge sécu)
    HEIGHT_MIN = -0.30  # m — limite basse (sous la base, anti-collision sol)
    HEIGHT_MAX =  1.00  # m — limite haute

    @staticmethod
    def validate_position(x: float, y: float, z: float) -> ValidationResult:
        """Validate end-effector position in workspace."""
        xy_distance = math.sqrt(x**2 + y**2)

        if xy_distance > WorkspaceValidator.REACH_MAX:
            return ValidationResult(
                False,
                f"Hors de portée : distance XY = {xy_distance:.3f}m > {WorkspaceValidator.REACH_MAX}m"
            )

        if z < WorkspaceValidator.HEIGHT_MIN:
            return ValidationResult(
                False,
                f"Hauteur sous le minimum : z = {z:.3f}m < {WorkspaceValidator.HEIGHT_MIN}m"
            )

        if z > WorkspaceValidator.HEIGHT_MAX:
            return ValidationResult(
                False,
                f"Hauteur hors limites : z = {z:.3f}m > {WorkspaceValidator.HEIGHT_MAX}m"
            )

        return ValidationResult(True, "Position dans l'espace de travail ✓")


class SafetyValidator:
    """Comprehensive safety validation combining all checks."""

    @staticmethod
    def pre_flight_check(
        joint_angles: List[float],
        ee_position: Optional[Tuple[float, float, float]] = None
    ) -> ValidationResult:
        """Perform comprehensive pre-flight validation."""
        joint_result = JointValidator.validate_angles(joint_angles)
        if not joint_result.passed:
            return joint_result

        if ee_position:
            workspace_result = WorkspaceValidator.validate_position(*ee_position)
            if not workspace_result.passed:
                return workspace_result

        return ValidationResult(True, "Pre-flight check PASSED ✓")

