import numpy as np
import math
from tqdm import tqdm, trange
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon


class UserInfo:
    """
    The representation of the user (position, angle, velocity and angular velocity).
    """

    def __init__(self, x, y, angle, v, w):
        self.x = x
        self.y = y
        self.angle = angle
        self.v = v
        self.w = w


class Space:
    """
    The representation of the physical space.
    We consider it as a polygon space with polygonal obstacles obstacle_list.
    Additionally, we record the position of the user (user_x, user_y), its angle (user_angle) and velocity (user_v) and angular velocity (user_w).
    """

    def __init__(self, border, raw_obstacle_list):
        self.border = [(t["x"], t["y"]) for t in border]
        self.obstacle_list = []
        for raw_obstacle in raw_obstacle_list:
            obstacle = [(t["x"], t["y"]) for t in raw_obstacle]
            self.add_obstacle(obstacle)
    def gethw(self):
        h,w = 0,0
        for i in range(len(self.border)):
            h = max(h,self.border[i][1])
            w = max(w,self.border[i][0])
        return h,w
    def add_obstacle(self, obstacle):
        self.obstacle_list.append(obstacle)

    def in_obstacle(self, x, y):
        if not Polygon(self.border).contains(Point(x, y)):
            return True
        for obstacle in self.obstacle_list:
            polygon = Polygon(obstacle)
            if polygon.contains(Point(x, y)):
                return True
        return False

    def closest_obstacle(self, x, y):
        min_dist = float("inf")
        closest_obstacle = None

        polygon = Polygon(self.border)
        for i in range(len(polygon.exterior.coords) - 1):
            p1 = Point(polygon.exterior.coords[i])
            p2 = Point(polygon.exterior.coords[i + 1])
            line = LineString([p1, p2])
            dist = line.distance(Point(x, y))
            if dist < min_dist:
                min_dist = dist
                closest_obstacle = polygon

        for obstacle in self.obstacle_list:
            polygon = Polygon(obstacle)
            dist = polygon.distance(Point(x, y))
            if dist < min_dist:
                min_dist = dist
                closest_obstacle = polygon

        return min_dist, closest_obstacle

    def nearest_obstacle_normal(self, user: UserInfo):
        """
        计算离用户最近的障碍物边线的法向量,朝向用户方向的反方向。

        参数:
        user (UserInfo): 用户信息

        返回:
        (float, float): 法向量的 x 和 y 分量
        """
        min_dist, closest_obstacle = self.closest_obstacle(user.x, user.y)

        # 如果没有找到最近的障碍物,则返回 (0, 0)
        if closest_obstacle is None:
            return 0, 0

        # 找到离用户最近的障碍物边线
        min_line_dist = float("inf")
        nearest_line = None
        for i in range(len(closest_obstacle.exterior.coords) - 1):
            p1 = Point(closest_obstacle.exterior.coords[i])
            p2 = Point(closest_obstacle.exterior.coords[i + 1])
            line = LineString([p1, p2])
            dist = line.distance(Point(user.x, user.y))
            if dist < min_line_dist:
                min_line_dist = dist
                nearest_line = line

        # 计算法向量
        dx = nearest_line.coords[1][0] - nearest_line.coords[0][0]
        dy = nearest_line.coords[1][1] - nearest_line.coords[0][1]
        normal_x = -dy
        normal_y = dx
        norm = np.sqrt(normal_x**2 + normal_y**2)
        normal_x /= norm
        normal_y /= norm

        # 确保法向量指向用户方向的反方向
        if normal_x * np.cos(user.angle) + normal_y * np.sin(user.angle) >= 0:
            normal_x = -normal_x
            normal_y = -normal_y

        return normal_x, normal_y
