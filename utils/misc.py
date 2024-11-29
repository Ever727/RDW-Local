from utils.space import *
import math


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
