# pick_place_state_machine.py — Architecture moderne avec smach-ros2 ou transitions
#!/usr/bin/env python3
"""
Machine d'état robuste pour pick & place industriel.
Utilise la bibliothèque 'transitions' (pip install transitions).
Chaque état peut gérer ses erreurs et déclencher des transitions.
"""

from transitions import Machine
from dataclasses import dataclass, field
from typing import Optional, Callable
import rclpy
from rclpy.node import Node


@dataclass
class PickPlaceContext:
    """Contexte partagé entre tous les états."""
    object_x: float = 0.0
    object_y: float = 0.0
    object_z: float = 0.0
    grasp_joints: Optional[list] = None
    pre_pick_joints: Optional[list] = None
    place_joints: Optional[list] = None
    retry_count: int = 0
    max_retries: int = 3
    last_error: str = ""
    gripper_closed: bool = False


class PickPlaceStateMachine:
    """
    Machine d'état pour pick & place robuste.
    
    Transitions possibles:
    idle → detecting_object → computing_ik → moving_home
         → pre_pick → grasping → lifting → pre_place
         → placing → releasing → success
    
    Tout état → error_recovery → idle (si récupérable)
    Tout état → abort (si critique)
    """
    
    STATES = [
        'idle',
        'detecting_object',
        'computing_ik',
        'moving_home',
        'moving_pre_pick',
        'grasping',
        'lifting',
        'moving_pre_place',
        'placing',
        'releasing',
        'success',
        'error_recovery',  # CRUCIAL: état de récupération
        'aborted',
    ]
    
    TRANSITIONS = [
        # Flux nominal
        {'trigger': 'start',         'source': 'idle',             'dest': 'detecting_object'},
        {'trigger': 'object_found',  'source': 'detecting_object', 'dest': 'computing_ik'},
        {'trigger': 'ik_solved',     'source': 'computing_ik',     'dest': 'moving_home'},
        {'trigger': 'at_home',       'source': 'moving_home',      'dest': 'moving_pre_pick'},
        {'trigger': 'at_pre_pick',   'source': 'moving_pre_pick',  'dest': 'grasping'},
        {'trigger': 'grasped',       'source': 'grasping',         'dest': 'lifting'},
        {'trigger': 'lifted',        'source': 'lifting',          'dest': 'moving_pre_place'},
        {'trigger': 'at_pre_place',  'source': 'moving_pre_place', 'dest': 'placing'},
        {'trigger': 'placed',        'source': 'placing',          'dest': 'releasing'},
        {'trigger': 'released',      'source': 'releasing',        'dest': 'success'},
        
        # Transitions d'erreur depuis TOUS les états mobiles
        {'trigger': 'error', 'source': [
            'detecting_object', 'computing_ik', 'moving_home',
            'moving_pre_pick', 'lifting', 'moving_pre_place', 'placing'
        ], 'dest': 'error_recovery'},
        
        # L'échec lors de la saisie est critique (objet peut être serré)
        {'trigger': 'error',         'source': 'grasping',    'dest': 'aborted'},
        
        # Récupération
        {'trigger': 'recovered',     'source': 'error_recovery', 'dest': 'idle'},
        {'trigger': 'unrecoverable', 'source': 'error_recovery', 'dest': 'aborted'},
        
        # Reset depuis n'importe où
        {'trigger': 'reset',         'source': '*',           'dest': 'idle'},
    ]
    
    def __init__(self, node: Node, context: PickPlaceContext):
        self.node = node
        self.ctx = context
        self.log = node.get_logger()
        
        self.machine = Machine(
            model=self,
            states=self.STATES,
            transitions=self.TRANSITIONS,
            initial='idle',
            after_state_change='_on_state_change'
        )
    
    def _on_state_change(self):
        self.log.info(f'[FSM] État: {self.state}')
    
    # ── Callbacks de récupération ─────────────────────────────────────────────
    
    def on_enter_error_recovery(self):
        """Tentative de récupération automatique."""
        self.ctx.retry_count += 1
        self.log.warn(
            f'Récupération d\'erreur (tentative {self.ctx.retry_count}/{self.ctx.max_retries})'
            f' — cause: {self.ctx.last_error}')
        
        if self.ctx.retry_count >= self.ctx.max_retries:
            self.log.error('Nombre max de tentatives atteint → ABORT')
            self.unrecoverable()
            return
        
        # Tenter d'ouvrir le gripper en premier (sécurité)
        # Puis retourner HOME
        self.log.info('Récupération: ouverture gripper + retour HOME')
        # Note: dans l'implémentation réelle, appeler les services async ici
        self.recovered()
    
    def on_enter_aborted(self):
        self.log.error(
            f'SÉQUENCE ABORTÉE. Historique: {self.ctx.last_error}')
        # Ici: déclencher une alarme, notifier l'opérateur, etc.
    
    def on_enter_success(self):
        self.log.info('✅ Pick & Place RÉUSSI')
        self.ctx.retry_count = 0
