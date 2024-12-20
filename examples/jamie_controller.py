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
        # Find the closest asteroid
        closest_ast = None
        closest_ast_dist = float('inf')
        for asteroid in game_state['asteroids']:
            dist = math.sqrt((asteroid['position'][0] - ship_state['position'][0])**2 +
                            (asteroid['position'][1] - ship_state['position'][1])**2)
            if dist < closest_ast_dist:
                closest_ast_dist = dist
                closest_ast = asteroid

        # Ship's position and heading
        ship_x, ship_y = ship_state['position']
        ship_heading_deg = ship_state['heading']
        bullet_speed = 300  # Approximate speed of the bullet (adjust as needed)

        # Asteroid's current position and velocity
        ast_x, ast_y = closest_ast['position']
        ast_vx, ast_vy = closest_ast['velocity']

        # Predict where the asteroid will be
        t = 0  # Initial guess for time
        for _ in range(10):  # Iterate to refine the prediction
            future_ast_x = ast_x + ast_vx * t
            future_ast_y = ast_y + ast_vy * t
            dist_to_future_ast = math.sqrt((future_ast_x - ship_x)**2 + (future_ast_y - ship_y)**2)
            t = dist_to_future_ast / bullet_speed

        # Future position of the asteroid
        future_ast_x = ast_x + ast_vx * t
        future_ast_y = ast_y + ast_vy * t

        # Calculate angle to the predicted position
        angle_to_ast_rad = math.atan2(future_ast_y - ship_y, future_ast_x - ship_x)
        angle_to_ast_deg = math.degrees(angle_to_ast_rad)

        # Calculate angle difference
        ship_heading_rad = math.radians(ship_heading_deg)
        angle_diff_rad = math.radians(angle_to_ast_deg) - ship_heading_rad
        angle_diff_rad = (angle_diff_rad + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
        angle_diff_deg = math.degrees(angle_diff_rad)

        # Set turn rate proportional to the angle difference
        turn_sensitivity = 5  # Adjust as needed
        turn_rate = angle_diff_deg * turn_sensitivity
        max_turn_rate = 170  # Maximum allowed turn rate
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
