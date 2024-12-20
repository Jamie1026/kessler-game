# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
import math


class JamieController(KesslerController):
    def __init__(self):
        """
        Any variables or initialization desired for the controller can be set up here
        """
        ...

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:
        """
        Method processed each time step by this controller to determine what control actions to take

        Arguments:
            ship_state (dict): contains state information for your own ship
            game_state (dict): contains state information for all objects in the game

        Returns:
            float: thrust control value
            float: turn-rate control value
            bool: fire control value. Shoots if true
            bool: mine deployment control value. Lays mine if true
        """
        #print(ship_state)
        #print()
        #print (game_state)
        #print()

        print(game_state['asteroids'][0]['position'][1])
        closest_ast = None
        closest_ast_dist_so_far = 100000000000
        for asteroid in game_state['asteroids']:
            ast_x = asteroid['position'][0]
            ast_y = asteroid['position'][1]
            ship_x = ship_state['position'][0]
            ship_y = ship_state['position'][1]
            dist_to_current_asteroid = math.sqrt((ast_x - ship_x) ** 2 + (ast_y - ship_y) ** 2)
            if dist_to_current_asteroid < closest_ast_dist_so_far:
                closest_ast_dist_so_far = dist_to_current_asteroid
                closest_ast = asteroid

        # Calculate angle to the closest asteroid in radians
        ship_x, ship_y = ship_state['position']
        ast_x, ast_y = closest_ast['position']
        angle_to_ast_rad = math.atan2(ast_y - ship_y, ast_x - ship_x)

        # Convert ship heading from degrees to radians for comparison
        ship_heading_deg = ship_state['heading']
        ship_heading_rad = math.radians(ship_heading_deg)

        # Calculate angle difference in radians
        angle_diff_rad = angle_to_ast_rad - ship_heading_rad
        angle_diff_rad = (angle_diff_rad + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

        # Convert angle difference back to degrees
        angle_diff_deg = math.degrees(angle_diff_rad)

        # Set turn rate proportional to the angle difference
        turn_sensitivity = 5  # Adjust as needed for responsiveness
        turn_rate = angle_diff_deg * turn_sensitivity

        # Clamp turn_rate to maximum allowed value if needed (e.g., ±170 degrees/second)
        max_turn_rate = 170  # Maximum allowed turn rate in degrees/second
        turn_rate = max(-max_turn_rate, min(turn_rate, max_turn_rate))

        # Set constant thrust
        thrust = 50

        # Always fire
        fire = True

        # Don't drop mines
        drop_mine = False

        return thrust, turn_rate, fire, drop_mine

    '''
    Goal: Aim at closest asteroid
    Find closest ast
        - We know where we are
        - We know where the other asteroids are
    '''

    @property
    def name(self) -> str:
        """
        Simple property used for naming controllers such that it can be displayed in the graphics engine

        Returns:
            str: name of this controller
        """
        return "Jamie Controller"

    # @property
    # def custom_sprite_path(self) -> str:
    #     return "Neo.png"
