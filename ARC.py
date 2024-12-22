from utils.space import *
import time
from utils.constants import *
from utils.misc import *
import math
from shapely.geometry import Point, LineString
import math
import numpy as np


def calc_move_with_gain(user, trans_gain, rot_gain, cur_gain_r, cur_direction, delta):
    x = user.x
    y = user.y
    dir = user.angle
    d_s = user.v
    d_dir = user.w

    d_s /= trans_gain
    d_dir /= rot_gain

    dir += d_dir
    if cur_gain_r != 0:
        dir += cur_direction * d_s / cur_gain_r
        dir = dir % (2 * math.pi)

    x += d_s * math.cos(dir)
    y += d_s * math.sin(dir)

    return UserInfo(x, y, dir, user.v, user.w)


def calc_min_angel_diff(angle1, angle2):
    return min(abs(angle1 - angle2), 2 * math.pi - abs(angle1 - angle2))


def calc_rotation_direction(target_angle, user_angle):
    target_angle_diff = target_angle - user_angle
    a = 1 if target_angle_diff > 0 else -1
    b = 1 if abs(target_angle_diff) < math.pi else -1
    return a * b


def distance_to_obstacle(user: UserInfo, space: Space, angle: float = 0):
    """
    计算从位置 p=(x,y) 出发，朝方向 theta 的最近障碍物的距离

    参数:
    user (UserInfo): 用户信息
    space (Space): 空间信息

    返回:
    float: 最近障碍物的距离
    """
    x, y = user.x, user.y
    theta = (user.angle + angle + 2 * math.pi) % (2 * math.pi)
    min_distance = float("inf")

    # 构建从当前位置出发的射线
    line = LineString([(x, y), (x + 10000 * np.cos(theta), y + 10000 * np.sin(theta))])

    # 检查边界
    intersections = [line.intersection(Polygon(space.border))]
    for intersection in intersections:
        if not intersection.is_empty:
            distance = intersection.length
            if distance < min_distance:
                min_distance = distance

    # 遍历所有障碍物
    for obstacle in space.obstacle_list:
        # 计算射线与障碍物的交点
        intersections = [line.intersection(Polygon(obstacle))]

        # 找到最近的交点
        for intersection in intersections:
            if not intersection.is_empty:
                distance = intersection.length
                if distance < min_distance:
                    min_distance = distance

    # 如果没有找到交点，说明没有障碍物
    if min_distance == float("inf"):
        return float("inf")
    else:
        return min_distance


def align(
    physical_user: UserInfo,
    virtual_user: UserInfo,
    physical_space: Space,
    virtual_space: Space,
):
    return (
        abs(
            distance_to_obstacle(virtual_user, virtual_space)
            - distance_to_obstacle(physical_user, physical_space)
        )
        + abs(
            distance_to_obstacle(virtual_user, virtual_space, math.pi / 2)
            - distance_to_obstacle(physical_user, physical_space, math.pi / 2)
        )
        + abs(
            distance_to_obstacle(virtual_user, virtual_space, -math.pi / 2)
            - distance_to_obstacle(physical_user, physical_space, -math.pi / 2)
        )
    )


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

# Global variables for ARC
prev_rota_alignment = -1.0
cur_rota_alignment = -1.0
prev_rota_gain = 1.0
last_reset_time = 0

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

    if cur_rota_alignment < 5:
        return 1, 1, INF_CUR_GAIN_R, 1
    else:
        trans_gain = clamp(
            phys_distance_front / virt_distance_front if virt_distance_front > 0 else MAX_TRANS_GAIN,
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
        misalign_right = phys_distance_right - virt_distance_right
        scale_factor = abs(min(1.0, max(misalign_left, misalign_right)))
        if scale_factor == 0:
            scale_factor = 1e-6

        rota_direction = 1 if misalign_left < misalign_right else -1

        cur_gain = max(MIN_CUR_GAIN_R, MIN_CUR_GAIN_R / scale_factor) * PIXEL

        print(
            f"Alignment: {cur_rota_alignment}, Trans gain: {trans_gain}, Rota gain: {rota_gain}, Cur gain: {cur_gain}, Rota direction: {rota_direction}"
        )
        return trans_gain, rota_gain, cur_gain, rota_direction
