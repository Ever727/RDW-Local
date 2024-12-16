import time
from utils.constants import *
from utils.space import *
from utils.misc import *
import math
import torch

# Global variables for S2C and S2O
previous_rotation = 0
temporary_target_use = False
temporary_target_position = (0, 0)

# Global variables for ARC
prev_rota_alignment = -1.0
cur_rota_alignment = -1.0
prev_rota_gain = 1.0
last_reset_time = 0


def calc_gain_generalized(
    user: UserInfo,
    delta: float,
    target_angle,
    target_distance,
):
    """
    Calculate gains for generalized redirected walking using the generalized redirected walking mechanisms.
    """
    global previous_rotation

    # Calculate rotation direction and curvature radius
    target_angle_diff = calc_min_angel_diff(user.angle, target_angle)
    rotation_direction = calc_rotation_direction(target_angle, user.angle)
    # k = 5  # Curvature radius scaling factor
    # curvature_radius = min(k * math.sin(target_angle_diff) * 100, MIN_CUR_GAIN_R)
    # if curvature_radius == 0:
    #     curvature_radius = INF_CUR_GAIN_R

    print(
        f"Target angle: {target_angle}, distance: {target_distance}, angle diff: {target_angle_diff}, rotation direction: {rotation_direction}"
    )
    # Base rotation gain: Apply minimal redirection even when standing
    if user.v <= 0.2:  # If user is standing
        baseline_rotation_gain = (
            0.5 * delta
        )  # Maximum rotation rate in degrees per second
    else:
        baseline_rotation_gain = 0

    # Linear movement gain: Steer to arc users towards the center
    if user.v > 0.2:  # If user is walking
        arc_radius = 7.5  # Minimum arc radius
        linear_movement_gain = min(
            (360 / (2 * math.pi * arc_radius)) * user.v * delta, 15 * delta
        )
    else:
        linear_movement_gain = 0

    # Angular movement gain: Adjust rotation based on angular velocity
    if abs(user.w) > math.radians(1.5) * delta:  # If user is turning
        angular_rate_scaling = 1.3 if rotation_direction > 0 else 0.85
        angular_movement_gain = min(angular_rate_scaling * user.w * delta, 30 * delta)
    else:
        angular_movement_gain = 0

    # Select the maximum gain for this frame
    selected_rotation_gain = math.radians(
        max(baseline_rotation_gain, linear_movement_gain, angular_movement_gain)
    )  # Convert to radians per second

    # Calculate rotation gain with dampening
    dampening_range = 160  # Dampening range in degrees per second
    if target_angle_diff < math.radians(dampening_range):
        dampening_gain = selected_rotation_gain * math.sin(
            target_angle * 90 / dampening_range
        )
    else:
        dampening_gain = selected_rotation_gain

    # Calculate rotation gain with threshold
    threshold_distance = 1.25 * 40  # Threshold distance in meters
    if target_distance < threshold_distance:
        threshold_gain = dampening_gain * target_distance / threshold_distance
    else:
        threshold_gain = dampening_gain

    # Apply smoothing to rotation gain
    s = 0.125  # Smoothing factor
    rotation_gain_smoothed = abs(previous_rotation * (1 - s) + threshold_gain * s)
    previous_rotation = rotation_gain_smoothed  # Update previous rotation

    # Calculate rotation gain rate
    basic_rotation_angle = (user.v / MAX_TRANS_GAIN) * delta / (MIN_CUR_GAIN_R / 100)
    print(
        f"Basic rotation angle: {basic_rotation_angle}, rotation gain: {rotation_gain_smoothed}"
    )
    if basic_rotation_angle == 0:
        curvature_radius = INF_CUR_GAIN_R
    else:
        curvature_radius = min(
            (MIN_CUR_GAIN_R * basic_rotation_angle / rotation_gain_smoothed),
            MIN_CUR_GAIN_R,
        )

    print(
        f"Rotation gain: {MAX_ROT_GAIN}, direction: {rotation_direction}, curvature radius: {curvature_radius}"
    )

    return MAX_TRANS_GAIN, MAX_ROT_GAIN, MIN_CUR_GAIN_R * PIXEL, rotation_direction


def calc_gain_S2C(user: UserInfo, physical_space: Space, delta: float):
    """
    Calculate gains for Steer-to-Center (S2C) using the generalized redirected walking mechanisms.
    """
    global temporary_target_use, temporary_target_position
    # Compute the center of the space
    center_x = sum([point[0] for point in physical_space.border]) / len(
        physical_space.border
    )
    center_y = sum([point[1] for point in physical_space.border]) / len(
        physical_space.border
    )

    # Calculate vector towards center
    dx = user.x - center_x
    dy = user.y - center_y
    center_angle = (math.atan2(dy, dx) + math.pi) % (2 * math.pi)
    center_distance = math.sqrt(dx**2 + dy**2)
    print(f"Center angle: {center_angle}, User angle: {user.angle}")
    # Angle difference between user's direction and target
    angle_diff = calc_min_angel_diff(center_angle, user.angle)

    if angle_diff > math.radians(160) and not temporary_target_use:
        temporary_target_use = True  # Set temporary target to center
        temporary_angle_1 = (user.angle + math.pi / 2) % (2 * math.pi)
        temporary_angle_2 = (user.angle - math.pi / 2 + 2 * math.pi) % (2 * math.pi)
        temporary_target_angle = (
            temporary_angle_1
            if calc_min_angel_diff(temporary_angle_1, center_angle)
            < calc_min_angel_diff(temporary_angle_2, center_angle)
            else temporary_angle_2
        )
        temporary_target_position = (
            user.x + 160 * math.cos(temporary_target_angle),
            user.y + 160 * math.sin(temporary_target_angle),
        )
        print(f"Temporary target set to {temporary_target_position}")
    elif angle_diff <= math.radians(160) and temporary_target_use:
        temporary_target_use = False  # Reset temporary target
        print("Temporary target reset")

    dx = user.x - temporary_target_position[0]
    dy = user.y - temporary_target_position[1]
    temporary_target_distance = math.sqrt(dx**2 + dy**2)
    temporary_target_angle = (math.atan2(dy, dx) + math.pi) % (2 * math.pi)

    target_angle = temporary_target_angle if temporary_target_use else center_angle
    target_distance = (
        temporary_target_distance if temporary_target_use else center_distance
    )

    return calc_gain_generalized(user, delta, target_angle, target_distance)


def calc_gain_S2O(user: UserInfo, physical_space: Space, delta: float):
    """
    Calculate gains for Steer-to-Orbit (S2O) using the generalized redirected walking mechanisms.
    """
    # Define orbit parameters
    orbit_radius = 200.0
    center_x = sum([point[0] for point in physical_space.border]) / len(
        physical_space.border
    )
    center_y = sum([point[1] for point in physical_space.border]) / len(
        physical_space.border
    )

    # Compute user's distance to center
    dx = user.x - center_x
    dy = user.y - center_y
    distance_to_center = math.sqrt(dx**2 + dy**2)
    tangent_angle = (math.atan2(dy, dx) + math.pi) % (2 * math.pi)

    # Calculate direction to orbit target
    if distance_to_center > orbit_radius:  # Outside orbit
        # Find the tangent point to the orbit
        target_distance = math.sqrt(distance_to_center**2 - orbit_radius**2)
        offset_angle = math.asin(orbit_radius / distance_to_center)
    else:  # Inside orbit
        target_distance = math.sqrt(
            distance_to_center**2 + orbit_radius**2 - distance_to_center * orbit_radius
        )
        offset_angle = math.asin(math.sin(math.pi / 3) * orbit_radius / target_distance)
    print(
        f"Tangent angle: {tangent_angle}, User angle: {user.angle}, Offset angle: {offset_angle}"
    )

    target_angle_op1 = (tangent_angle - offset_angle + 2 * math.pi) % (2 * math.pi)
    target_angle_op2 = (tangent_angle + offset_angle) % (2 * math.pi)
    target_angle = (
        target_angle_op1
        if calc_min_angel_diff(target_angle_op1, user.angle)
        < calc_min_angel_diff(target_angle_op2, user.angle)
        else target_angle_op2
    )

    return calc_gain_generalized(user, delta, target_angle, target_distance)


class FixedNormal(torch.distributions.Normal):
    def log_probs(self, actions):
        return super().log_prob(actions).sum(-1, keepdim=True)

    def entrop(self):
        return super().entropy().sum(-1)

    def mode(self):
        return self.mean


def split_action(action):
    gt, gr, gc = action[0]
    gt = 1.060 + 0.2 * gt
    gr = 1.145 + 0.345 * gr
    gc = 0.05 * gc
    return gt, gr, gc


def calc_gain_RL(user: UserInfo, physical_space: Space, delta: float, model_path: str):
    """
    Calculate gains for Reinforcement Learning (RL) using the generalized redirected walking mechanisms.
    """
    # Load the RL model
    model = torch.load(model_path)
    height, width = 200, 200
    obs = []
    obs.extend(
        10
        * [(user.x) / height, (user.y) / width, (user.angle + math.pi) / (2 * math.pi)]
    )
    observation = torch.Tensor(obs)
    observation = torch.Tensor(observation).unsqueeze(0)
    with torch.no_grad():
        _value, action_mean, action_log_std = model.act(observation)
        dist = FixedNormal(action_mean, action_log_std)
        action = dist.mode()
    gt, gr, gc = split_action(action)
    gt, gr, gc = gt.item(), gr.item(), 1 / gc.item()

    if gc > 0:
        direction = 1
    else:
        direction = -1
        gc = -gc
    print(
        f"Translational gain: {gt}, Rotational gain: {gr}, Curvature gain: {gc}, Direction: {direction}"
    )
    return gt, gr, gc, direction


def calc_gain_ARC(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
):
    global prev_rota_alignment, cur_rota_alignment, prev_rota_gain

    phys_distance_front = distance_to_obstacle(physical_user, physical_space)
    phys_distance_right = distance_to_obstacle(
        physical_user, physical_space, math.pi / 2
    )
    phys_distance_left = distance_to_obstacle(
        physical_user, physical_space, -math.pi / 2
    )

    virt_distance_front = distance_to_obstacle(virtual_user, virtual_space)
    virt_distance_right = distance_to_obstacle(virtual_user, virtual_space, math.pi / 2)
    virt_distance_left = distance_to_obstacle(virtual_user, virtual_space, -math.pi / 2)

    cur_rota_alignment = align(
        physical_user, virtual_user, physical_space, virtual_space
    )
    print(
        f"phys_distance_front: {phys_distance_front}, virt_distance_front: {virt_distance_front}, cur_rota_alignment: {cur_rota_alignment}"
    )
    print(
        f"phys_distance_right: {phys_distance_right}, virt_distance_right: {virt_distance_right}"
    )
    print(
        f"phys_distance_left: {phys_distance_left}, virt_distance_left: {virt_distance_left}"
    )

    if cur_rota_alignment == 0:
        return 1, 1, INF_CUR_GAIN_R, 1
    else:
        trans_gain = clamp(
            phys_distance_front / virt_distance_front,
            MIN_TRANS_GAIN,
            MAX_TRANS_GAIN,
        )

        if cur_rota_alignment > prev_rota_alignment:
            rota_gain = (
                1 - ROTA_GAIN_SMOOTHING
            ) * prev_rota_gain + ROTA_GAIN_SMOOTHING * MIN_ROT_GAIN
        else:
            rota_gain = (
                1 - ROTA_GAIN_SMOOTHING
            ) * prev_rota_gain + ROTA_GAIN_SMOOTHING * MAX_ROT_GAIN

        prev_rota_alignment = cur_rota_alignment
        prev_rota_gain = rota_gain

        misalign_left = phys_distance_left - virt_distance_left
        misalign_right = virt_distance_right - phys_distance_right
        scale_factor = abs(min(1.0, max(misalign_left, misalign_right)))
        if scale_factor == 0:
            scale_factor = 1e-6

        rota_direction = 1 if misalign_left < misalign_right else -1

        cur_gain = max(MIN_CUR_GAIN_R, MIN_CUR_GAIN_R / scale_factor) * PIXEL

        print(
            f"Alignment: {cur_rota_alignment}, Trans gain: {trans_gain}, Rota gain: {rota_gain}, Cur gain: {cur_gain}, Rota direction: {rota_direction}"
        )
        return trans_gain, rota_gain, cur_gain, rota_direction


def calc_gain(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
    algorithm="ARC",
):
    """
    Return three gains and the direction (+1 or -1) when cur_gain used. Implement your own logic here.
    """
    if algorithm == "S2C":
        return calc_gain_S2C(physical_user, physical_space, delta)
    elif algorithm == "S2O":
        return calc_gain_S2O(physical_user, physical_space, delta)
    elif algorithm == "RL":
        return calc_gain_RL(physical_user, physical_space, delta, "models/5900.pth")
    elif algorithm == "ARC":
        return calc_gain_ARC(
            physical_user, virtual_user, physical_space, virtual_space, delta
        )
    else:
        return MAX_TRANS_GAIN, MAX_ROT_GAIN, INF_CUR_GAIN_R, 1


def reset_ARC(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
):
    """
    Return new UserInfo when RESET. Implement your own logic here.
    """
    sample_directions = [(2 * math.pi) / SAMPLE_RATE * i for i in range(SAMPLE_RATE)]
    closest_wall_normal = physical_space.nearest_obstacle_normal(physical_user)
    virt_dist_front = distance_to_obstacle(virtual_user, virtual_space)

    best_loss = float("inf")
    best_angle = 0
    best_under_loss = float("inf")
    best_under_angle = 0

    for angle in sample_directions:
        direction = (
            math.cos(angle + physical_user.angle),
            math.sin(angle + physical_user.angle),
        )
        direction_arr = np.array(direction)
        closest_wall_normal_arr = np.array(closest_wall_normal)

        if np.dot(direction_arr, closest_wall_normal_arr) < 1e-6:
            continue

        phys_dist_front = distance_to_obstacle(physical_user, physical_space, angle)
        loss = abs(phys_dist_front - virt_dist_front)

        if loss < best_loss:
            best_loss = loss
            best_angle = angle

        if phys_dist_front < virt_dist_front:
            if loss < best_under_loss:
                best_under_loss = loss
                best_under_angle = angle

    if best_loss == float("inf"):
        best_angle = best_under_angle

    physical_user.angle = (best_angle + physical_user.angle) % (2 * math.pi)

    return physical_user


def update_reset(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
    algorithm="ARC",
):
    """
    Return new UserInfo when RESET. Implement your RESET logic here.
    """
    if algorithm == "ARC":
        return reset_ARC(
            physical_user, virtual_user, physical_space, virtual_space, delta
        )
    else:
        physical_user.angle = (physical_user.angle + math.pi) % (2 * math.pi)
        return physical_user


def need_reset_ARC(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
):
    global last_reset_time

    distance, object = physical_space.closest_obstacle(physical_user.x, physical_user.y)
    print(f"Distance to closest obstacle: {distance}, Object: {object}")
    if distance < 0.7 * PIXEL:
        current_time = time.time()
        print(f"Current time: {current_time}, Last reset time: {last_reset_time}")
        if current_time - last_reset_time > 0.3:
            last_reset_time = current_time
            return True
    return False


def judge_reset(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
    delta: float,
    algorithm="ARC",
):
    """
    Return True if need reset, otherwise False. Implement your own logic here.
    """
    if algorithm == "ARC":
        return need_reset_ARC(
            physical_user, virtual_user, physical_space, virtual_space, delta
        )
    else:
        return False
