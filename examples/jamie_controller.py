# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
import math

def wrapped_distance(x1, y1, x2, y2, map_size):
    """Calculate the shortest distance between two points considering screen wrapping."""
    width, height = map_size
    dx = min(abs(x1 - x2), width - abs(x1 - x2))
    dy = min(abs(y1 - y2), height - abs(y1 - y2))
    return math.sqrt(dx**2 + dy**2)

def wrapped_position(x, y, map_size):
    """Wrap the position of an object to keep it inside the map."""
    width, height = map_size
    return x % width, y % height

def time_to_collision_wrapped(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius, map_size):
    x_s, y_s = ship_pos
    x_a, y_a = asteroid_pos
    v_ax, v_ay = asteroid_velocity
    r_s = ship_radius
    r_a = asteroid_radius

    # Calculate effective positions considering wrapping
    wrapped_positions = [
        (x_a + dx, y_a + dy)
        for dx in [-map_size[0], 0, map_size[0]]
        for dy in [-map_size[1], 0, map_size[1]]
    ]

    best_time = float('inf')

    for wrapped_pos in wrapped_positions:
        x_a, y_a = wrapped_pos

        # Quadratic coefficients
        a = v_ax**2 + v_ay**2
        if a == 0:
            # Stationary asteroid case
            distance = wrapped_distance(x_s, y_s, x_a, y_a, map_size)
            if distance <= r_s + r_a:
                return 0  # Immediate collision
            continue

        b = 2 * ((x_a - x_s) * v_ax + (y_a - y_s) * v_ay)
        c = (wrapped_distance(x_s, y_s, x_a, y_a, map_size))**2 - (r_s + r_a)**2

        # Discriminant
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            continue  # No collision

        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)

        # Update the best collision time if positive
        if t1 > 0:
            best_time = min(best_time, t1)
        elif t2 > 0:
            best_time = min(best_time, t2)

    return best_time

def prioritize_imminent_collision(ship_state: Dict, game_state: Dict) -> Dict:
    ship_pos = ship_state['position']
    ship_radius = ship_state['radius']
    map_size = game_state['map_size']
    
    imminent_asteroid = None
    shortest_time = float('inf')
    
    for asteroid in game_state['asteroids']:
        asteroid_pos = asteroid['position']
        asteroid_velocity = asteroid['velocity']
        asteroid_radius = asteroid['radius']
        
        # Ignore asteroids leaving the screen imminently
        next_x, next_y = (
            asteroid_pos[0] + asteroid_velocity[0] / 30,
            asteroid_pos[1] + asteroid_velocity[1] / 30,
        )
        if not (0 <= next_x <= map_size[0] and 0 <= next_y <= map_size[1]):
            continue  # Exclude asteroids imminently leaving the screen
        
        # Consider wrapping for collision time
        t = time_to_collision_wrapped(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius, map_size)
        
        if t < shortest_time:
            shortest_time = t
            imminent_asteroid = asteroid
    
    return imminent_asteroid

def time_to_collision(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius):
    x_s, y_s = ship_pos
    x_a, y_a = asteroid_pos
    v_ax, v_ay = asteroid_velocity
    r_s = ship_radius
    r_a = asteroid_radius

    # Quadratic coefficients
    a = v_ax**2 + v_ay**2
    if a == 0:
        # Asteroid is stationary; check if it's already on a collision course
        distance = math.sqrt((x_a - x_s)**2 + (y_a - y_s)**2)
        if distance <= r_s + r_a:
            return 0  # Immediate collision
        else:
            return float('inf')  # No collision as it's stationary and out of range
    #If asteroid go offscreen track asteroid screen wrapping
    b = 2 * ((x_a - x_s) * v_ax + (y_a - y_s) * v_ay)
    c = (x_a - x_s)**2 + (y_a - y_s)**2 - (r_s + r_a)**2

    # Discriminant
    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        return float('inf')  # No collision
    else:
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        
        # Return the smallest positive time
        if t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        else:
            return float('inf')  # Both times are negative (past collision)

def prioritize_imminent_collision_OLD(ship_state: Dict, game_state: Dict) -> Dict:
    ship_pos = ship_state['position']
    ship_radius = ship_state['radius']
    
    imminent_asteroid = None
    shortest_time = float('inf')
    
    for asteroid in game_state['asteroids']:
        asteroid_pos = asteroid['position']
        asteroid_velocity = asteroid['velocity']
        asteroid_radius = asteroid['radius']
        
        t = time_to_collision(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius)
        
        if t < shortest_time:
            shortest_time = t
            imminent_asteroid = asteroid
    
    return imminent_asteroid

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
        most_imminent_ast = prioritize_imminent_collision(ship_state, game_state)
        if most_imminent_ast is not None:
            closest_ast = most_imminent_ast
        # Ship's position and heading
        ship_x, ship_y = ship_state['position']
        ship_heading_deg = ship_state['heading']
        bullet_speed = 800  # Approximate speed of the bullet

        # Asteroid's current position and velocity
        ast_x, ast_y = closest_ast['position']
        ast_vx, ast_vy = closest_ast['velocity']

        # Predict where the asteroid will be
        t = 0  # Initial guess for time
        for _ in range(5):  # Iterate to refine the prediction
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
        turn_sensitivity = 30  # Adjust as needed
        turn_rate = angle_diff_deg * turn_sensitivity
        max_turn_rate = 180  # Maximum allowed turn rate
        if abs(turn_rate) <= 180:
            fire = True
        else:
            fire = False
        turn_rate = max(-max_turn_rate, min(turn_rate, max_turn_rate))

        # Set constant thrust
        thrust = 50

        # Always fire
        #fire = True

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
