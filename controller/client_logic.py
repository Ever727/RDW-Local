from utils.constants import *
from utils.space import *
from utils.misc import *
import math

previous_rotation = 0
temporary_target_use = False
temporary_target_position = (0, 0)


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
        curvature_radius = (
            MIN_CUR_GAIN_R
            * basic_rotation_angle
            / (basic_rotation_angle + rotation_gain_smoothed)
        )

    print(
        f"Rotation gain: {MAX_ROT_GAIN}, direction: {rotation_direction}, curvature radius: {curvature_radius}"
    )

    return MAX_TRANS_GAIN, MAX_ROT_GAIN, curvature_radius, rotation_direction


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


def calc_gain(user: UserInfo, physical_space: Space, delta: float, algorithm="S2C"):
    """
    Return three gains and the direction (+1 or -1) when cur_gain used. Implement your own logic here.
    """
    if algorithm == "S2C":
        return calc_gain_S2C(user, physical_space, delta)
    elif algorithm == "S2O":
        return calc_gain_S2O(user, physical_space, delta)
    else:
        return MAX_TRANS_GAIN, MAX_ROT_GAIN, INF_CUR_GAIN_R, 1


def update_reset(user: UserInfo, physical_space: Space, delta: float):
    """
    Return new UserInfo when RESET. Implement your RESET logic here.
    """
    user.angle = (user.angle + math.pi) % (2 * math.pi)
    return user
