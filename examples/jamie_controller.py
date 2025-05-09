# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
import math
import numpy as np

print_explanation = True
def log_explanation(message: str):
    if print_explanation:
        print(message)
    else:
        with open('explanations_jamie.txt', 'a+') as file:
            file.write(message + '\n')

def wrapped_distance(x1, y1, x2, y2, map_size):
    """Calculate the shortest distance between two points considering screen wrapping."""
    width, height = map_size
    dx = min(abs(x1 - x2), width - abs(x1 - x2))
    dy = min(abs(y1 - y2), height - abs(y1 - y2))
    return math.sqrt(dx**2 + dy**2)

def time_to_collision_wrapped(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius, map_size):
    # This function takes in a velocity with units of pixels/second, and returns time in seconds
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
            distance = math.sqrt((x_s - x_a)**2 + (y_s - y_a)**2)
            if distance <= r_s + r_a:
                return 0  # Immediate collision
            continue

        b = 2 * ((x_a - x_s) * v_ax + (y_a - y_s) * v_ay)
        c = c = (x_s - x_a)**2 + (y_s - y_a)**2 - (r_s + r_a)**2

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

#start

def predict_imminent_collision(ship_state: Dict, game_state: Dict, delta_time: float) -> bool:
    """Predict if any asteroid will collide with the ship within the next frame"""
    ship_pos = ship_state['position']
    ship_radius = 20
    map_size = game_state['map_size']
    prediction_time = 2*delta_time
    
    # Convert ship's speed and heading to velocity components
    ship_speed = ship_state['speed']
    ship_heading_rad = math.radians(ship_state['heading'])
    ship_vel = (
        ship_speed * math.cos(ship_heading_rad),
        ship_speed * math.sin(ship_heading_rad)
    )
    
    for asteroid in game_state['asteroids']:
        # Current asteroid position and velocity
        ast_pos = asteroid['position']
        ast_vel = asteroid['velocity']
        ast_radius = asteroid['radius']
        
        # Calculate relative velocity (ship + asteroid)
        rel_vel = (
            ast_vel[0] - ship_vel[0], 
            ast_vel[1] - ship_vel[1]
        ) # pixels per sec
        
        future_x = ast_pos[0] + ast_vel[0]*delta_time*2
        future_y = ast_pos[1] + ast_vel[1]*delta_time*2
        if (future_x - ship_pos[0])**2 + (future_y - ship_pos[1])**2 <= (ship_radius + ast_radius)**2:
            return True
            
    return False

def prioritize_imminent_collision(ship_state: Dict, game_state: Dict) -> Dict:
    ship_pos = ship_state['position']
    ship_radius = ship_state['radius']
    map_size = game_state['map_size']
    bullet_speed = 800  # Speed of the ship's bullets


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

        # Calculate collision time considering wrapping
        t_col = time_to_collision_wrapped(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius, map_size)

        # Future position of the target asteroid
        ast_x, ast_y = asteroid['position']
        ast_vx, ast_vy = asteroid['velocity']
        t_int = 0
        for _ in range(5):  # Refine prediction iteratively
            future_ast_x = ast_x + ast_vx * t_int
            future_ast_y = ast_y + ast_vy * t_int
            dist_to_future_ast = math.sqrt((future_ast_x - ship_pos[0])**2 + (future_ast_y - ship_pos[1])**2)
            t_int = dist_to_future_ast / bullet_speed

        # Predict future asteroid position at collision time
        future_ast_x = asteroid_pos[0] + asteroid_velocity[0] * t_int
        future_ast_y = asteroid_pos[1] + asteroid_velocity[1] * t_int

        # Check if the predicted future position is within screen bounds
        if not (0 <= future_ast_x <= map_size[0] and 0 <= future_ast_y <= map_size[1]):
            continue  # Skip asteroids that will be off-screen at collision time

        # Verify that the bullet can realistically intercept the asteroid
        dist_to_future_ast = math.sqrt((future_ast_x - ship_pos[0])**2 + (future_ast_y - ship_pos[1])**2)
        bullet_travel_time = dist_to_future_ast / bullet_speed

        # Update the closest imminent asteroid if this one is a better candidate
        if t_col < shortest_time:
            shortest_time = t_col
            imminent_asteroid = asteroid

    return imminent_asteroid

class JamieController(KesslerController):
    def __init__(self):
        self.prev_lives = None
        self.last_mine_time = -10  # Initialize to ensure first mine can be placed
        self.mine_cooldown = 3.0  # Minimum time between mines (seconds)

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:
        # Ship state and properties
        ship_x, ship_y = ship_state['position']
        ship_heading_deg = ship_state['heading']
        ship_heading_rad = math.radians(ship_heading_deg)
        bullet_speed = 800
        current_time = game_state['time']
        delta_time = game_state['delta_time']
        #yeah
        if ship_state['lives_remaining'] == 3:
            thrust = 0
        elif ship_state['lives_remaining'] == 2:
            thrust = 0
        else:
            thrust = 0

        # Mine deployment logic - predict collisions before they happen
        drop_mine = False
        if (ship_state['can_deploy_mine'] and ship_state['mines_remaining'] > 0 and current_time - self.last_mine_time >= self.mine_cooldown):
            # Check for imminent collision in next frame
            if predict_imminent_collision(ship_state, game_state, delta_time):
                drop_mine = True
                print("Dropping a mine!")
                self.last_mine_time = current_time
                #collison_time = 666 frame
            # Also keep the hit detection as backup
            #elif (self.prev_lives is not None and 
                  #ship_state['lives_remaining'] < self.prev_lives):
                #drop_mine = True
                #self.last_mine_time = current_time

        # Update previous lives
        self.prev_lives = ship_state['lives_remaining']

        # Asteroid targeting logic
        most_imminent_ast = prioritize_imminent_collision(ship_state, game_state)
        if most_imminent_ast is not None:
            we_gon_shot_this = most_imminent_ast
        else:
            min_rotation = float('inf')
            we_gon_shot_this = None

            for asteroid in game_state['asteroids']:
                ast_x, ast_y = asteroid['position']
                ast_vx, ast_vy = asteroid['velocity']

                # Predict the asteroid's future position
                t = 0
                for _ in range(5):  # Refine prediction iteratively
                    future_ast_x = ast_x + ast_vx * t
                    future_ast_y = ast_y + ast_vy * t
                    dist_to_future_ast = math.sqrt((future_ast_x - ship_x)**2 + (future_ast_y - ship_y)**2)
                    t = dist_to_future_ast / bullet_speed

                # Calculate angle to the predicted position
                angle_to_ast_rad = math.atan2(future_ast_y - ship_y, future_ast_x - ship_x)
                angle_diff_rad = (angle_to_ast_rad - ship_heading_rad + math.pi) % (2 * math.pi) - math.pi
                angle_diff_deg = abs(math.degrees(angle_diff_rad))

                # Check if this asteroid requires less rotation
                if angle_diff_deg < min_rotation:
                    min_rotation = angle_diff_deg
                    we_gon_shot_this = asteroid

        if we_gon_shot_this is None:
            return 0, 0, False, drop_mine

        # Calculate firing solution
        ast_x, ast_y = we_gon_shot_this['position']
        ast_vx, ast_vy = we_gon_shot_this['velocity']
        t = 0
        for _ in range(5):
            future_ast_x = ast_x + ast_vx * t
            future_ast_y = ast_y + ast_vy * t
            dist_to_future_ast = math.sqrt((future_ast_x - ship_x)**2 + (future_ast_y - ship_y)**2)
            t = dist_to_future_ast / bullet_speed

        future_ast_x = ast_x + ast_vx * (t + 1/30)
        future_ast_y = ast_y + ast_vy * (t + 1/30)

        angle_to_ast_rad = math.atan2(future_ast_y - ship_y, future_ast_x - ship_x)
        angle_diff_rad = (angle_to_ast_rad - ship_heading_rad + math.pi) % (2 * math.pi) - math.pi
        angle_diff_deg = math.degrees(angle_diff_rad)

        turn_magic_num = 30
        turn_rate = angle_diff_deg * turn_magic_num
        max_turn_rate = 180
        if abs(turn_rate) <= 180:
            fire = True
        else:
            if angle_diff_deg > 180/10 and not (0 <= ship_state['bullets_remaining'] <= 35): # SPAMMMMMM since we got bullets and because we can
                fire = True
            else:
                fire = False
        turn_rate = max(-max_turn_rate, min(turn_rate, max_turn_rate)) # clamppity damp
        
        if ship_state["is_respawning"]:
            fire = False 
   

        # RAM mode when out of bullets
        if not ship_state['can_fire'] and ship_state['bullets_remaining'] == 0:
            my_team = ship_state['team']
            enemy_ships = [
                s for s in game_state['ships']
                if s['team'] != my_team and not s['is_respawning']
            ]
            if enemy_ships:
                target = min(
                    enemy_ships,
                    key=lambda s: math.hypot(s['position'][0] - ship_x, s['position'][1] - ship_y)
                )
                target_x, target_y = target['position']

                dx = target_x - ship_x
                dy = target_y - ship_y
                desired_angle_rad = math.atan2(dy, dx)
                desired_angle_deg = math.degrees(desired_angle_rad)

                angle_diff = (desired_angle_deg - ship_heading_deg + 180) % 360 - 180

                turn = max(min(angle_diff * 5, 180), -180)
                thrust = ship_state['thrust_range'][1]
                return thrust, turn, False, drop_mine

        return thrust, turn_rate, fire, drop_mine

    @property
    def name(self) -> str:
        return "Jamie Controller"

    @property
    def custom_sprite_path(self) -> str:
        return "prideship.png"