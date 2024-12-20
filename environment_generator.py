import json
import math
import random

# 定义环境大小
ENVIRONMENT_SIZE = 60

# 定义最大障碍物数量
MAX_OBSTACLES = 12

# 定义最小和最大障碍物尺寸
MIN_OBSTACLE_SIZE = 2
MAX_OBSTACLE_SIZE = 6

border_phys = []
border_virt = []
obstacles_phys = []
obstacles_virt = []
poi = []
user_phys = None
user_virt = None


# 按顺时针排序顶点
def sort_vertices_clockwise(vertices):
    """
    按顺时针方向对多边形顶点进行排序。
    :param vertices: 顶点列表，格式 [{"x": int, "y": int}, ...]
    :return: 排序后的顶点列表
    """
    center = {
        "x": sum(v["x"] for v in vertices) / len(vertices),
        "y": sum(v["y"] for v in vertices) / len(vertices),
    }
    return sorted(
        vertices,
        key=lambda v: math.atan2(v["y"] - center["y"], v["x"] - center["x"]),
    )


# 生成矩形边界
def generate_rectangular_border():
    """
    生成一个矩形边界
    :return: 矩形的顶点列表，顺时针排序
    """
    x1, y1 = random.randint(0, ENVIRONMENT_SIZE // 2), random.randint(
        0, ENVIRONMENT_SIZE // 2
    )
    x2, y2 = random.randint(ENVIRONMENT_SIZE // 2, ENVIRONMENT_SIZE), random.randint(
        ENVIRONMENT_SIZE // 2, ENVIRONMENT_SIZE
    )
    return [
        {"x": x1, "y": y1},  # 左上角
        {"x": x2, "y": y1},  # 右上角
        {"x": x2, "y": y2},  # 右下角
        {"x": x1, "y": y2},  # 左下角
    ]


# 生成矩形障碍物
def generate_rectangular_obstacles(existing_obstacles, border, max_obstacles):
    """
    生成矩形障碍物，基于一个基准点生成其余顶点
    :param existing_obstacles: 已存在的障碍物列表
    :param border: 环境边界
    :param max_obstacles: 最大障碍物数量
    :return: 矩形障碍物列表
    """
    obstacles = []
    while len(obstacles) < max_obstacles:
        # 随机选择基准点
        base_point = {
            "x": random.randint(0, ENVIRONMENT_SIZE - MAX_OBSTACLE_SIZE),
            "y": random.randint(0, ENVIRONMENT_SIZE - MAX_OBSTACLE_SIZE),
        }

        # 随机生成矩形的宽度和高度
        width = random.randint(MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)
        height = random.randint(MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)

        # 依据基准点生成矩形的其余顶点
        obstacle = [
            {"x": base_point["x"], "y": base_point["y"]},  # 左上角
            {"x": base_point["x"] + width, "y": base_point["y"]},  # 右上角
            {"x": base_point["x"] + width, "y": base_point["y"] + height},  # 右下角
            {"x": base_point["x"], "y": base_point["y"] + height},  # 左下角
        ]

        # 确保顶点不会超出环境边界
        for v in obstacle:
            v["x"] = min(max(0, v["x"]), ENVIRONMENT_SIZE)
            v["y"] = min(max(0, v["y"]), ENVIRONMENT_SIZE)

        # 检查障碍物是否与现有障碍物重叠或超出边界
        overlap = False
        for o in existing_obstacles:
            if any(is_point_in_polygon(v, o) for v in obstacle):
                overlap = True
                break

        if not overlap and all(is_point_in_polygon(v, border) for v in obstacle):
            obstacles.append(obstacle)

    return obstacles


# 检查点是否在多边形内
def is_point_in_polygon(point, polygon):
    """
    检查点是否在多边形内，使用改进的射线法。
    :param point: 要检查的点，格式 {"x": int, "y": int}
    :param polygon: 多边形顶点列表，格式 [{"x": int, "y": int}, ...]
    :return: 如果点在多边形内，返回 True，否则返回 False
    """
    x, y = point["x"], point["y"]
    inside = False
    n = len(polygon)

    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]

        # 检查点是否在线段上（特别处理）
        if min(p1["y"], p2["y"]) <= y <= max(p1["y"], p2["y"]) and min(
            p1["x"], p2["x"]
        ) <= x <= max(p1["x"], p2["x"]):
            # 计算斜率是否相等来判断是否在边上
            if (p2["y"] - p1["y"]) * (x - p1["x"]) == (y - p1["y"]) * (
                p2["x"] - p1["x"]
            ):
                return True

        # 检查交点是否在射线右侧
        if (p1["y"] > y) != (p2["y"] > y):
            intersect_x = (p2["x"] - p1["x"]) * (y - p1["y"]) / (
                p2["y"] - p1["y"]
            ) + p1["x"]
            if x < intersect_x:
                inside = not inside

    return inside


# 生成以中心点为基础的障碍物
def generate_obstacles_centered(existing_obstacles, border, max_obstacles):
    obstacles = []
    while len(obstacles) < max_obstacles:
        # 随机选择一个中心点
        center = {
            "x": random.randint(
                MIN_OBSTACLE_SIZE, ENVIRONMENT_SIZE - MIN_OBSTACLE_SIZE
            ),
            "y": random.randint(
                MIN_OBSTACLE_SIZE, ENVIRONMENT_SIZE - MIN_OBSTACLE_SIZE
            ),
        }

        # 确保中心点在边界内
        if not is_point_in_polygon(center, border):
            continue

        # 在中心点附近生成顶点
        num_vertices = random.randint(3, 6)
        radius = random.randint(MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)  # 障碍物大小
        vertices = []
        for _ in range(num_vertices):
            angle = random.uniform(0, 2 * math.pi)
            x = center["x"] + int(radius * math.cos(angle))
            y = center["y"] + int(radius * math.sin(angle))
            vertices.append(
                {
                    "x": max(0, min(ENVIRONMENT_SIZE, x)),
                    "y": max(0, min(ENVIRONMENT_SIZE, y)),
                }
            )

        # 按顺时针排序顶点
        obstacle = sort_vertices_clockwise(vertices)

        # 检查新障碍物是否与现有障碍物重叠或超出边界
        overlap = False
        for o in existing_obstacles:
            if any(
                is_point_in_polygon({"x": v["x"], "y": v["y"]}, o) for v in obstacle
            ):
                overlap = True
                break
        if not overlap and all(
            is_point_in_polygon({"x": v["x"], "y": v["y"]}, border) for v in obstacle
        ):
            obstacles.append(obstacle)

    return obstacles


# 生成边界
def generate_border():
    num_vertices = random.randint(4, 8)
    vertices = []
    for _ in range(num_vertices):
        x = random.randint(0, ENVIRONMENT_SIZE)
        y = random.randint(0, ENVIRONMENT_SIZE)
        vertices.append({"x": x, "y": y})
    return sort_vertices_clockwise(vertices)


# 生成用户初始位置
def generate_user_position(border, obstacles):
    user = None
    while user is None:
        user = {
            "x": random.randint(
                MIN_OBSTACLE_SIZE, ENVIRONMENT_SIZE - MIN_OBSTACLE_SIZE
            ),
            "y": random.randint(
                MIN_OBSTACLE_SIZE, ENVIRONMENT_SIZE - MIN_OBSTACLE_SIZE
            ),
            "angle": random.uniform(0, 2 * math.pi),
            "v": 0,
            "w": 0,
        }
        if all(
            not is_point_in_polygon({"x": user["x"], "y": user["y"]}, o)
            for o in obstacles
        ) and is_point_in_polygon({"x": user["x"], "y": user["y"]}, border):
            break
        else:
            user = None
    return user


# 生成最终数据
def generate_environment():
    global border_phys, border_virt, obstacles_phys, obstacles_virt, poi, user_phys, user_virt

    # border_phys = generate_border()
    # border_virt = generate_border()
    # obstacles_phys = generate_obstacles_centered([], border_phys, MAX_OBSTACLES)
    # obstacles_virt = generate_obstacles_centered([], border_virt, MAX_OBSTACLES)
    border_phys = generate_rectangular_border()
    border_virt = generate_rectangular_border()
    obstacles_phys = generate_rectangular_obstacles([], border_phys, MAX_OBSTACLES)
    obstacles_virt = generate_rectangular_obstacles([], border_virt, MAX_OBSTACLES)
    user_phys = generate_user_position(border_phys, obstacles_phys)
    user_virt = generate_user_position(border_virt, obstacles_virt)
    poi.append({"x": user_virt["x"], "y": user_virt["y"]})

    environment = {
        "border_phys": border_phys,
        "border_virt": border_virt,
        "obstacles_phys": obstacles_phys,
        "obstacles_virt": obstacles_virt,
        "poi": poi,
        "initial_user_phys": user_phys,
        "initial_user_virt": user_virt,
        "walk_speed": 10,
        "turn_speed": 5,
    }

    return environment


# 生成并保存 JSON 文件
environment = generate_environment()
with open("base.json", "w") as f:
    json.dump(environment, f, indent=2)

print("JSON 文件已生成: base.json")
