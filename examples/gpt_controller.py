import math
import numpy as np

def angle_wrap_deg(angle):
    """Wrap angle to [-180, 180]"""
    a = (angle + 180) % 360 - 180
    return a

def dist(p1, p2, map_size):
    """Euclidean distance considering wraparound."""
    dx = min(abs(p2[0] - p1[0]), map_size[0] - abs(p2[0] - p1[0]))
    dy = min(abs(p2[1] - p1[1]), map_size[1] - abs(p2[1] - p1[1]))
    return math.hypot(dx, dy)

def nearest_asteroid(ship_pos, asteroids, map_size):
    best = None
    best_dist = float('inf')
    for a in asteroids:
        d = dist(ship_pos, a['position'], map_size)
        # Heuristic: favor bigger asteroids but also closer ones
        score = d - 40 * a['size']
        if score < best_dist:
            best_dist = score
            best = a
    return best

def wrap_position(pos, map_size):
    x = pos[0] % map_size[0]
    y = pos[1] % map_size[1]
    return (x, y)

def predict_asteroid_position(asteroid, t, map_size):
    x = asteroid['position'][0] + asteroid['velocity'][0] * t
    y = asteroid['position'][1] + asteroid['velocity'][1] * t
    return wrap_position((x,y), map_size)

def lead_shot_solution(ship_pos, bullet_speed, asteroid, map_size):
    # Solve: |ship_pos + v_bullet * t - (ast_pos + v_asteroid * t)| = 0
    # Use quadratic formula for t
    sx, sy = ship_pos
    ax, ay = asteroid['position']
    bvx = asteroid['velocity'][0]
    bvy = asteroid['velocity'][1]

    # Adjust for wrapping (find the shortest vector)
    dx = ax - sx
    dy = ay - sy
    if abs(dx) > map_size[0] / 2:
        if dx > 0: dx -= map_size[0]
        else: dx += map_size[0]
    if abs(dy) > map_size[1] / 2:
        if dy > 0: dy -= map_size[1]
        else: dy += map_size[1]
    rel_a_pos = np.array([dx, dy])
    rel_a_vel = np.array([bvx, bvy])

    a = np.dot(rel_a_vel, rel_a_vel) - bullet_speed**2
    b = 2 * np.dot(rel_a_pos, rel_a_vel)
    c = np.dot(rel_a_pos, rel_a_pos)

    disc = b * b - 4 * a * c
    if disc < 0:
        return None  # can't hit
    sqrt_disc = math.sqrt(disc)
    t1 = (-b + sqrt_disc) / (2 * a)
    t2 = (-b - sqrt_disc) / (2 * a)
    t = min([x for x in [t1, t2] if x > 0], default=None)
    if t is None or t > 2.0:  # only shoot if hit occurs soon
        return None
    target = predict_asteroid_position(asteroid, t, map_size)
    dx = target[0] - sx
    dy = target[1] - sy
    # account for wrapping
    if abs(dx) > map_size[0] / 2:
        if dx > 0: dx -= map_size[0]
        else: dx += map_size[0]
    if abs(dy) > map_size[1] / 2:
        if dy > 0: dy -= map_size[1]
        else: dy += map_size[1]
    angle_deg = math.degrees(math.atan2(dy, dx))
    return angle_deg

class Controller:
    @property
    def name(self) -> str:
        return "gpt cont"
    def actions(self, ship_state, game_state):
        # Constants
        FIELD = game_state['map_size']
        BULLET_SPEED = 800.0 # px/sec
        SHIP_POS = ship_state['position']
        SHIP_HEADING = ship_state['heading']
        IS_INVINC = ship_state.get('invincibility_time', 0) > 0 if 'invincibility_time' in ship_state else False

        # Step 1: Threat assessment and evade if needed
        danger = False
        evade_vec = np.array([0.0, 0.0])
        for asteroid in game_state['asteroids']:
            # Predict where asteroid will be in 0.5 and 1.0 seconds
            for dt in [0.5, 1.0]:
                pred_ast = predict_asteroid_position(asteroid, dt, FIELD)
                d = dist(SHIP_POS, pred_ast, FIELD)
                if d < asteroid['radius'] + ship_state['radius'] + 20:  # allow safety margin
                    # Compute direction to thrust away
                    dx = SHIP_POS[0] - pred_ast[0]
                    dy = SHIP_POS[1] - pred_ast[1]
                    # Correct for wrapping
                    if abs(dx) > FIELD[0]/2:
                        dx -= FIELD[0] * np.sign(dx)
                    if abs(dy) > FIELD[1]/2:
                        dy -= FIELD[1] * np.sign(dy)
                    evade_vec += np.array([dx, dy])
                    danger = True

        # Step 2: Decide action
        thrust = 0.0
        turn_rate = 0.0
        fire = False
        drop_mine = False

        # Step 2a: Evade threats, if any
        if danger and np.linalg.norm(evade_vec) > 1e-3:
            evade_angle = math.degrees(math.atan2(evade_vec[1], evade_vec[0]))
            angle_error = angle_wrap_deg(evade_angle - SHIP_HEADING)
            # Turn toward evade heading
            # Max 180 deg/sec turn rate, smooth turning
            max_turn = ship_state.get('turn_rate_range', (-180,180))[1]
            turn_rate = max(-max_turn, min(angle_error * 4.0, max_turn))  # 4x error P-controller
            thrust = 400.0  # quickly move away
            # No firing while evading for safety
        else:
            # Step 2b: Target asteroids
            if game_state['asteroids']:
                target = nearest_asteroid(SHIP_POS, game_state['asteroids'], FIELD)
                # Compute lead angle to target
                solution_deg = lead_shot_solution(SHIP_POS, BULLET_SPEED, target, FIELD)
                if solution_deg is not None:
                    angle_error = angle_wrap_deg(solution_deg - SHIP_HEADING)
                    # P-controller for smooth turn
                    max_turn = ship_state.get('turn_rate_range', (-180,180))[1]
                    turn_rate = max(-max_turn, min(angle_error * 4.0, max_turn))
                    # Thrust moderately toward target
                    if abs(angle_error) < 15:
                        thrust = 240.0
                        # Step 3: FIRE if lined up and allowed
                        if ship_state['can_fire'] and (ship_state['bullets_remaining'] != 0):
                            fire = True
                    else:
                        thrust = 100.0  # gently circle toward line-up
                else:
                    # Can't hit, rotate very slightly and move forward
                    turn_rate = 50.0
                    thrust = 100.0
            else:
                # No targets: move in a spiral
                turn_rate = 60.0
                thrust = 120.0

            # Step 4: MINES (basic version)
            # Drop a mine if a large asteroid will pass near current spot in 3 seconds, and can deploy
            if ship_state.get('can_deploy_mine', False) and ship_state.get('mines_remaining', 0) != 0:
                for ast in game_state['asteroids']:
                    future_pos = predict_asteroid_position(ast, 3.0, FIELD)
                    if dist(SHIP_POS, future_pos, FIELD) < 150:  # within blast radius minus fudge
                        drop_mine = True
                        break

        return thrust, turn_rate, fire, drop_mine