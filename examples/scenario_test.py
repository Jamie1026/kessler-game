# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

import time
#from (file name without .py) import (name of controller class inside the file whatever that means)
from main_controller_akila import AkilaController
from galaxy_brain_controller import GalaxyBrain
from kesslergame import Scenario, KesslerGame, GraphicsType
from test_controller import TestController
from jamie_controller import JamieController
from graphics_both import GraphicsBoth
from neo_controller import BabyNeoController
#from  kesslergame.controller_gamepad import GamepadController
from adversarial_scenarios_for_jie import *
from custom_scenarios import *
from xfc_2023_replica_scenarios import *
#from gpt_controller import Controller

xfc2023 = [
     ex_adv_four_corners_pt1,
     ex_adv_four_corners_pt2,
     ex_adv_asteroids_down_up_pt1,
     ex_adv_asteroids_down_up_pt2,
     ex_adv_direct_facing,
     ex_adv_two_asteroids_pt1,
     ex_adv_two_asteroids_pt2,
     ex_adv_ring_pt1,
     adv_random_big_1,
     adv_random_big_2,
     adv_random_big_3,
     adv_random_big_4,
     adv_multi_wall_bottom_hard_1,
     adv_multi_wall_right_hard_1,
     adv_multi_ring_closing_left,
     adv_multi_ring_closing_right,
     adv_multi_two_rings_closing,
     avg_multi_ring_closing_both2,
     adv_multi_ring_closing_both_inside,
     adv_multi_ring_closing_both_inside_fast
 ]

xfc2024 = [
    adv_random_small_1,
    adv_random_small_1_2,
    adv_multi_wall_left_easy,
    adv_multi_four_corners,
    adv_multi_wall_top_easy,
    adv_multi_2wall_closing,
    adv_wall_bottom_staggered,
    adv_multi_wall_right_hard,
    adv_moving_corridor_angled_1,
    adv_moving_corridor_angled_1_mines,
    adv_multi_ring_closing_left,
    adv_multi_ring_closing_left2,
    adv_multi_ring_closing_both2,
    adv_multi_ring_closing_both_inside_fast,
    adv_multi_two_rings_closing
]

custom_scenarios = [
    #target_priority_optimization1,
    #closing_ring_scenario,
    #easy_closing_ring_scenario,
    #more_intense_closing_ring_scenario,
    #rotating_square_scenario,
    #rotating_square_2_overlap,
    #falling_leaves_scenario,
    #zigzag_motion_scenario,
    #shearing_pattern_scenario,
    #super_hard_wrap,
    #wonky_ring,
    #moving_ring_scenario,
    #shifting_square_scenario,
    #delayed_closing_ring_scenario,
    #spiral_assault_scenario,
    #dancing_ring,
    #dancing_ring_2,
    #intersecting_lines_scenario,
    #exploding_grid_scenario,
    #grid_formation_explosion_scenario,
    #aspect_ratio_grid_formation_scenario,
    #adv_asteroid_stealing,
    #wrapping_nightmare,
    #wrapping_nightmare_fast,
    #purgatory,
    #cross,
    #fight_for_asteroid,
    #shot_pred_test,
    #shredder,
    diagonal_shredder,
    out_of_bound_mine,
    explainability_1,
    explainability_2,
    split_forecasting,
    minefield_maze_scenario,
    wrap_collision_test
]

# Define game scenario
my_test_scenario = Scenario(name='Test Scenario',
                            num_asteroids=5,
                            #asteroid_states=[
                            #    {'position': (800, 400), 'angle': 0, 'speed': 200, 'size': 4},
                            #    {'position': (800, 100), 'angle': 0, 'speed': 200, 'size': 4}
                            #],
                            ship_states=[
                                {'position': (300, 400), 'angle': 90, 'lives': 300, 'team': 1, "mines_remaining": 3},
                                {'position': (400, 600), 'angle': 90, 'lives': 3, 'team': 2, "mines_remaining": 3},
                            ],
                            map_size=(1000, 700),
                            time_limit=60,
                            ammo_limit_multiplier=0,
                            stop_if_no_ammo=False)

# Define Game Settings
game_settings = {'perf_tracker': True,
                 'graphics_type': GraphicsType.Tkinter,
                 'realtime_multiplier': 1,
                 'graphics_obj': None,
                 'frequency': 30}

game = KesslerGame(settings=game_settings)  # Use this to visualize the game scenario
# game = TrainerEnvironment(settings=game_settings)  # Use this for max-speed, no-graphics simulation

# Evaluate the game
pre = time.perf_counter()


for sc in xfc2024 :
    score, perf_data = game.run(scenario=sc, controllers=[JamieController(), BabyNeoController()])

# Print out some general info about the result
print('Scenario eval time: '+str(time.perf_counter()-pre))
print(score.stop_reason)
print('Asteroids hit: ' + str([team.asteroids_hit for team in score.teams]))
print('Deaths: ' + str([team.deaths for team in score.teams]))
print('Accuracy: ' + str([team.accuracy for team in score.teams]))
print('Mean eval time: ' + str([team.mean_eval_time for team in score.teams]))
