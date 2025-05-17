# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
import math
import numpy as np

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

mine_preventative_lookahead_frames = 8

# === Fuzzy Variable Definitions ===

# Distance (0 to 1000 px)
distance = ctrl.Antecedent(np.arange(0, 1001, 1), 'distance')
# Speed (-240 to 240 px/s)
speed = ctrl.Antecedent(np.arange(-240, 241, 1), 'speed')
# Thrust (-480 to 480 px/s²)
thrust = ctrl.Consequent(np.arange(-480, 481, 1), 'thrust')

# Distance memberships
distance['very_close'] = fuzz.trapmf(distance.universe, [0, 0, 40, 80])
distance['close'] = fuzz.trimf(distance.universe, [60, 120, 180])
distance['medium'] = fuzz.trimf(distance.universe, [160, 350, 550])
distance['far'] = fuzz.trapmf(distance.universe, [500, 700, 1000, 1000])

# Speed memberships
speed['reverse'] = fuzz.trapmf(speed.universe, [-240, -240, -200, -140])
speed['normal_reverse'] = fuzz.trimf(speed.universe, [-180, -120, -60])
speed['zero'] = fuzz.trimf(speed.universe, [-80, 0, 80])
speed['normal_forward'] = fuzz.trimf(speed.universe, [60, 120, 180])
speed['forward'] = fuzz.trapmf(speed.universe, [140, 180, 240, 240])

# Thrust memberships
thrust['strong_reverse'] = fuzz.trapmf(thrust.universe, [-480, -480, -360, -240])
thrust['normal_reverse'] = fuzz.trimf(thrust.universe, [-300, -180, -60])
thrust['coast'] = fuzz.trimf(thrust.universe, [-80, 0, 80])
thrust['normal_forward'] = fuzz.trimf(thrust.universe, [60, 180, 300])
thrust['strong_forward'] = fuzz.trapmf(thrust.universe, [240, 360, 480, 480])

# === Fuzzy Rules ===
rules = [
    ctrl.Rule(distance['far'] & speed['reverse'], thrust['strong_forward']),
    ctrl.Rule(distance['far'] & speed['normal_reverse'], thrust['strong_forward']),
    ctrl.Rule(distance['far'] & speed['zero'], thrust['strong_forward']),
    ctrl.Rule(distance['far'] & speed['normal_forward'], thrust['normal_forward']),
    ctrl.Rule(distance['far'] & speed['forward'], thrust['normal_forward']),

    ctrl.Rule(distance['medium'] & speed['reverse'], thrust['strong_forward']),
    ctrl.Rule(distance['medium'] & speed['zero'], thrust['strong_forward']),
    ctrl.Rule(distance['medium'] & speed['normal_forward'], thrust['strong_forward']),
    ctrl.Rule(distance['medium'] & speed['forward'], thrust['normal_forward']),

    ctrl.Rule(distance['close'] & speed['normal_forward'], thrust['coast']), #what
    ctrl.Rule(distance['close'] & speed['forward'], thrust['coast']), #what
    ctrl.Rule(distance['close'] & speed['zero'], thrust['coast']),
    ctrl.Rule(distance['close'] & speed['normal_reverse'], thrust['normal_forward']),

    ctrl.Rule(distance['very_close'] & speed['forward'], thrust['strong_reverse']),
    ctrl.Rule(distance['very_close'] & speed['normal_forward'], thrust['normal_reverse']),
    ctrl.Rule(distance['very_close'] & speed['zero'], thrust['strong_reverse']),
    ctrl.Rule(distance['very_close'] & speed['reverse'], thrust['coast']),
]

# === Build the Control System ===
thrust_ctrl = ctrl.ControlSystem(rules)
thrust_sim = ctrl.ControlSystemSimulation(thrust_ctrl)

# === Plotting Memberships (Optional, for tuning) ===
#fig, axs = plt.subplots(3, 1, figsize=(12, 12))
#distance.view(ax=axs[0])
#axs[0].set_title('Distance Membership Functions')

#speed.view(ax=axs[1])
#axs[1].set_title('Speed Membership Functions')

#thrust.view(ax=axs[2])
#axs[2].set_title('Thrust Membership Functions')

#plt.tight_layout()
#plt.show()


print_explanation = True

last_message = ""

def log_explanation(message: str):
    global last_message
    if last_message == message:
        return
    else:
        if print_explanation:
            print(message)
        else:
            with open('explanations_jamie.txt', 'a+') as file:
                file.write(message + '\n')
        last_message = message

def wrapped_distance(x1, y1, x2, y2, map_size):
    width, height = map_size
    dx = min(abs(x1 - x2), width - abs(x1 - x2))
    dy = min(abs(y1 - y2), height - abs(y1 - y2))
    return math.sqrt(dx**2 + dy**2)

def time_to_collision_wrapped(ship_pos, ship_radius, asteroid_pos, asteroid_velocity, asteroid_radius, map_size):
    x_s, y_s = ship_pos
    v_ax, v_ay = asteroid_velocity
    r_s = ship_radius
    r_a = asteroid_radius

    wrapped_positions = [
        (asteroid_pos[0] + dx, asteroid_pos[1] + dy)
        for dx in [-map_size[0], 0, map_size[0]]
        for dy in [-map_size[1], 0, map_size[1]]
    ]

    best_time = float('inf')

    for x_a, y_a in wrapped_positions:
        a = v_ax**2 + v_ay**2
        if a == 0:
            distance = math.sqrt((x_s - x_a)**2 + (y_s - y_a)**2)
            if distance <= r_s + r_a:
                return 0
            continue

        b = 2 * ((x_a - x_s) * v_ax + (y_a - y_s) * v_ay)
        c = (x_s - x_a)**2 + (y_s - y_a)**2 - (r_s + r_a)**2
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            continue

        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        if t1 > 0:
            best_time = min(best_time, t1)
        elif t2 > 0:
            best_time = min(best_time, t2)

    return best_time

def canonicalize_asteroid(asteroid, game_time, map_size):
    # Take me back to the start babyyyyyy
    x, y = asteroid['position']
    vx, vy = asteroid['velocity']
    width, height = map_size
    x0 = (x - vx * game_time) % width
    y0 = (y - vy * game_time) % height
    return (round(x0, 5), round(y0, 5))

def predict_imminent_collision(ship_state: Dict, game_state: Dict, delta_time: float) -> bool:
    ship_pos = ship_state['position']
    ship_radius = 20
    map_size = game_state['map_size']
    ship_speed = ship_state['speed']
    ship_heading_rad = math.radians(ship_state['heading'])
    ship_vel = (
        ship_speed * math.cos(ship_heading_rad),
        ship_speed * math.sin(ship_heading_rad)
    )

    for asteroid in game_state['asteroids']:
        ast_pos = asteroid['position']
        ast_vel = asteroid['velocity']
        ast_radius = asteroid['radius']
        #rel_vel = (ast_vel[0] - ship_vel[0], ast_vel[1] - ship_vel[1])
        future_x = ast_pos[0]
        future_y = ast_pos[1]
        for _ in range(mine_preventative_lookahead_frames):
            future_x += ast_vel[0] * delta_time
            future_y += ast_vel[1] * delta_time
            if (future_x - ship_pos[0])**2 + (future_y - ship_pos[1])**2 <= (ship_radius + ast_radius)**2:
                return True
        

    return False

def prioritize_imminent_collision(ship_state: Dict, game_state: Dict) -> Dict:
    ship_pos = ship_state['position']
    ship_radius = ship_state['radius']
    map_size = game_state['map_size']
    bullet_speed = 800
    imminent_asteroid = None
    shortest_time = float('inf')

    for asteroid in game_state['asteroids']:
        t_col = time_to_collision_wrapped(ship_pos, ship_radius, asteroid['position'], asteroid['velocity'], asteroid['radius'], map_size)
        if t_col < shortest_time:
            shortest_time = t_col
            imminent_asteroid = asteroid

    return imminent_asteroid

class JamieController(KesslerController):
    def __init__(self):
        self.prev_lives = None
        self.last_mine_time = -10
        self.mine_cooldown = 3.0
        self.asteroids_targeted = {}
        self.fire_this_fram = False
        self.fire_next_fram = False

        #make better to win 
    
    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:
        if game_state["time"] == 0: #Tim's hacky code.
            self.prev_lives = None
            self.last_mine_time = -10
            self.mine_cooldown = 3.0
            self.asteroids_targeted = {}
            self.fire_this_fram = False
            self.fire_next_fram = False
        ship_x, ship_y = ship_state['position'] #(log_explanation = [position])
        ship_heading_deg = ship_state['heading'] 
        ship_heading_rad = math.radians(ship_heading_deg)
        bullet_speed = 800
        current_time = game_state['time']
        delta_time = game_state['delta_time']
        map_size = game_state['map_size']
        self.fire_this_fram = self.fire_next_fram
        log_explanation(f"Frame {game_state['sim_frame']}")
        # Expire old targeting info
        self.asteroids_targeted = {
            k: v for k, v in self.asteroids_targeted.items() if v > current_time
        }

        thrust = 0
        turn = 0
        fire = False
        drop_mine = False
         
        # Mine drop logic
        drop_mine = False
        if (ship_state['can_deploy_mine'] and ship_state['mines_remaining'] > 0 and current_time - self.last_mine_time >= self.mine_cooldown):
            if predict_imminent_collision(ship_state, game_state, delta_time):
                log_explanation("Damage incoming, dropping preventitive mine")
                drop_mine = True
                self.last_mine_time = current_time
        dont_spray = False
        self.prev_lives = ship_state['lives_remaining']
        target_asteroid = None
        if ship_state['bullets_remaining'] != 0:
            most_imminent_ast = prioritize_imminent_collision(ship_state, game_state)
            for asteroid in game_state['asteroids']:
                canon_key = canonicalize_asteroid(asteroid, current_time, map_size)
                if canon_key in self.asteroids_targeted:
                    continue # don't waste bullet, bad for environment badddd

                ast_x, ast_y = asteroid['position']
                ast_vx, ast_vy = asteroid['velocity']
                t = 0
                for _ in range(5):
                    future_x = ast_x + ast_vx * t
                    future_y = ast_y + ast_vy * t
                    dist = math.sqrt((future_x - ship_x)**2 + (future_y - ship_y)**2)
                    t = dist / bullet_speed

                future_x = ast_x + ast_vx * (t + 1/30)
                future_y = ast_y + ast_vy * (t + 1/30)
                angle_to = math.atan2(future_y - ship_y, future_x - ship_x)
                angle_diff = ((angle_to - ship_heading_rad + math.pi) % (2 * math.pi)) - math.pi
                angle_diff_deg = abs(math.degrees(angle_diff))
                if asteroid is most_imminent_ast:
                    target_asteroid = (asteroid, angle_diff_deg, t, canon_key, future_x, future_y)
                    break
                if target_asteroid is None or angle_diff_deg < target_asteroid[1]:
                    target_asteroid = (asteroid, angle_diff_deg, t, canon_key, future_x, future_y)

        my_team = ship_state['team']
        enemies = [s for s in game_state['ships'] if s['team'] != my_team]
        if enemies and enemies[0]['lives_remaining'] < ship_state['lives_remaining']:
            log_explanation(f"I have {ship_state['lives_remaining']} lives, which is more than my opponent's {enemies[0]['lives_remaining']} lives. Good time for RAM mode maybe...")
            greater_lives = True
        else:
            greater_lives = False
        #if ((target_asteroid is None and not self.fire_this_fram) or ship_state['lives_remaining'] >= 3)*0  and greater_lives:
        if (greater_lives and ship_state['is_respawning']): 
            log_explanation("We're doing RAM mode!")
            ram_mode = True
        else:
            log_explanation("Not doing RAM mode")
            ram_mode = False

        if target_asteroid:
            asteroid, angle_diff_deg, intercept_time, canon_key, future_x, future_y = target_asteroid

            angle_to = math.atan2(future_y - ship_y, future_x - ship_x)
            angle_diff = ((angle_to - ship_heading_rad + math.pi) % (2 * math.pi)) - math.pi
            angle_diff_deg = math.degrees(angle_diff)

            turn = max(-180, min(angle_diff_deg * 30, 180))
            self.fire_next_fram = abs(angle_diff_deg) < 6# or (angle_diff_deg > 18 and ship_state['bullets_remaining'] > 35)

            if abs(angle_diff_deg) < 6:
                log_explanation('I locked onto an asteroid!')
                self.asteroids_targeted[canon_key] = current_time + intercept_time # secs
            elif abs(angle_diff_deg) < 18:
                dont_spray = True

        # RAM mode
        #ram_mode= True 
        if ram_mode:
            drop_mine = False
            fire = False
            my_team = ship_state['team']
            enemies = [s for s in game_state['ships'] if s['team'] != my_team]

            if enemies:
                try:
                    # Find closest enemy
                    closest = min(
                        enemies,
                        key=lambda s: wrapped_distance(ship_x, ship_y, s['position'][0], s['position'][1], map_size)
                    )
                    target_x, target_y = closest['position']

                    # Calculate turning angle
                    dx = target_x - ship_x
                    dy = target_y - ship_y
                    desired = math.degrees(math.atan2(dy, dx))
                    angle_diff = ((desired - ship_heading_deg + 180) % 360) - 180
                    turn = max(-180, min(angle_diff * 5, 180))

                    # Fuzzy logic inputs
                    distance_val = wrapped_distance(ship_x, ship_y, target_x, target_y, map_size)
                    speed_val = ship_state['speed']

                    # Apply fuzzy logic
                    thrust_sim.input['distance'] = max(min(distance_val, 1000), 0)
                    thrust_sim.input['speed'] = max(min(speed_val, 240), -240)
                    thrust_sim.compute()
                    fuzzy_thrust = thrust_sim.output['thrust']

                    log_explanation(f"[RAM] Distance: {distance_val:.1f}, Speed: {speed_val:.1f} => Thrust: {fuzzy_thrust:.1f}, Turn: {turn:.1f}")
                    return fuzzy_thrust, turn, False, drop_mine
                except Exception as e:
                    log_explanation(f"fuzzy ram error: {e}")
                    return 0, 0, False, False
            else:
                return 0, 0, False, False



        if self.fire_this_fram is True and not ship_state['is_respawning']:
            fire = True
        else:
            fire = False
        if ship_state['can_fire'] and not (0 <= ship_state['bullets_remaining'] <= 35) and not self.fire_next_fram:
            if not ship_state['is_respawning'] and not dont_spray:
                fire = True
            else:
                fire = False
        return thrust, turn, fire, drop_mine

    @property
    def name(self) -> str:
        return "Jamie Controller"


    @property
    def custom_sprite_path(self) -> str:
        return "prideship.png"