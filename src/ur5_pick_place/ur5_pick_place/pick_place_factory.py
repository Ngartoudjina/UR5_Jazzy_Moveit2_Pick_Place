# pick_place_factory.py — Point d'entrée unique
#!/usr/bin/env python3
"""
Factory qui remplace les 10 fichiers pick_place_*.py
par une seule classe paramétrable.
"""

from enum import Enum

class ExecutionBackend(Enum):
    MOVEIT2_PYTHON    = "moveit2_python"     # moveit_py (Jazzy)
    MOVEIT2_ACTION    = "moveit2_action"     # MoveGroup action (Humble+)
    ROS2_CONTROL      = "ros2_control"       # FollowJointTrajectory direct
    HYBRID            = "hybrid"             # Fallback automatique

class PickPlaceFactory:
    """
    Crée le bon contrôleur selon l'environnement détecté.
    Plus besoin de choisir le bon fichier à lancer.
    """
    
    @staticmethod
    def create(node, backend: ExecutionBackend = ExecutionBackend.HYBRID):
        if backend == ExecutionBackend.HYBRID:
            backend = PickPlaceFactory._detect_best_backend(node)
        
        node.get_logger().info(f'Backend sélectionné: {backend.value}')
        
        if backend == ExecutionBackend.MOVEIT2_PYTHON:
            from ur5_pick_place.backends.moveit_py_backend import MoveItPyBackend
            return MoveItPyBackend(node)
        elif backend == ExecutionBackend.MOVEIT2_ACTION:
            from ur5_pick_place.backends.moveit_action_backend import MoveItActionBackend
            return MoveItActionBackend(node)
        else:
            from ur5_pick_place.backends.ros2_control_backend import Ros2ControlBackend
            return Ros2ControlBackend(node)
    
    @staticmethod
    def _detect_best_backend(node) -> ExecutionBackend:
        """Détecte automatiquement le meilleur backend disponible."""
        import subprocess
        
        # Vérifier si moveit_py est disponible (Jazzy+)
        try:
            import moveit  # noqa
            return ExecutionBackend.MOVEIT2_PYTHON
        except ImportError:
            pass
        
        # Vérifier si /move_action est disponible
        result = subprocess.run(
            ['ros2', 'action', 'info', '/move_action'],
            capture_output=True, timeout=3.0)
        if result.returncode == 0:
            return ExecutionBackend.MOVEIT2_ACTION
        
        return ExecutionBackend.ROS2_CONTROL
