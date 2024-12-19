import json
import math
import random
import sys

from shapely import Point, Polygon


def is_outside_border(point, border):
    """
    判断点是否在边界外，使用虚拟边界的坐标范围。
    """
    x, y = point["x"], point["y"]
    min_x = min(p["x"] for p in border)
    max_x = max(p["x"] for p in border)
    min_y = min(p["y"] for p in border)
    max_y = max(p["y"] for p in border)
    return not (min_x <= x <= max_x and min_y <= y <= max_y)


def is_in_obstacle(point, obstacle_list):
    x, y = point["x"], point["y"]
    for obstacle in obstacle_list:
        min_x = min(p["x"] for p in obstacle)
        max_x = max(p["x"] for p in obstacle)
        min_y = min(p["y"] for p in obstacle)
        max_y = max(p["y"] for p in obstacle)
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return True
    return False


def random_point_in_polygon(polygon):
    """
    随机生成一个在多边形内的点。
    """
    min_x = min(p["x"] for p in polygon)
    max_x = max(p["x"] for p in polygon)
    min_y = min(p["y"] for p in polygon)
    max_y = max(p["y"] for p in polygon)
    x, y = random.uniform(min_x, max_x), random.uniform(min_y, max_y)
    return x, y


def move_along_angle(x, y, angle, distance):
    """
    根据当前位置、角度和移动距离，计算新位置。
    """
    new_x = x + distance * math.cos(angle)
    new_y = y + distance * math.sin(angle)
    return {"x": new_x, "y": new_y}


def adjust_angle(current_angle):
    """
    随机调整角度，范围 0 到 2*pi。
    """
    return random.uniform(0, 2 * math.pi)


def line_intersect(p1, p2, q1, q2):
    """
    计算两条线段 (p1->p2) 和 (q1->q2) 的交点
    """

    def det(a, b, c, d):
        return a * d - b * c

    x1, y1 = p1["x"], p1["y"]
    x2, y2 = p2["x"], p2["y"]
    x3, y3 = q1["x"], q1["y"]
    x4, y4 = q2["x"], q2["y"]

    denom = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4)
    if denom == 0:
        return None  # 平行，无交点

    px = det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / denom
    py = det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / denom

    # 检查交点是否在线段范围内
    if (
        min(x1, x2) <= px <= max(x1, x2)
        and min(y1, y2) <= py <= max(y1, y2)
        and min(x3, x4) <= px <= max(x3, x4)
        and min(y3, y4) <= py <= max(y3, y4)
    ):
        return {"x": px, "y": py}
    return None


def find_collision_point(start, end, border):
    """
    找出从 start 到 end 与边界的交点
    """
    for i in range(len(border)):
        border_start = border[i]
        border_end = border[(i + 1) % len(border)]  # 形成闭合边界
        intersection = line_intersect(start, end, border_start, border_end)
        if intersection:
            return intersection
    return end  # 如果未找到交点，返回终点（备用）


def process_file(base_path, file_path):
    """
    主函数：根据起始点和边界计算路径，并存储结果。
    """
    try:
        # 读取 JSON 文件
        with open(base_path, "r", encoding="utf-8") as file:
            data = json.load(file)

        # 提取初始位置和角度
        current_position = data["initial_user_virt"]
        angle = current_position["angle"]
        border = data["border_virt"]
        obstacles = data["obstacles_virt"]

        # 现实环境
        current_position_phys = data["initial_user_phys"]
        border_phys = data["border_phys"]
        obstacles_phys = data["obstacles_phys"]

        # 随机化初始位置
        current_position_phys["x"], current_position_phys["y"] = (
            random_point_in_polygon(border_phys)
        )
        while is_outside_border(current_position_phys, border_phys) or is_in_obstacle(
            current_position_phys, obstacles_phys
        ):
            current_position_phys["x"], current_position_phys["y"] = (
                random_point_in_polygon(border_phys)
            )

        current_position_phys["angle"] = random.uniform(0, 2 * math.pi)
        data["initial_user_phys"] = current_position_phys

        print("初始物理位置：")
        print(
            f"x={current_position_phys['x']:.2f}, y={current_position_phys['y']:.2f}, angle={current_position_phys['angle']:.2f}"
        )

        current_position["x"], current_position["y"] = random_point_in_polygon(border)
        while is_outside_border(current_position, border) or is_in_obstacle(
            current_position, obstacles
        ):
            current_position["x"], current_position["y"] = random_point_in_polygon(
                border
            )
        data["initial_user_virt"] = current_position

        print("初始虚拟位置：")
        print(
            f"x={current_position['x']:.2f}, y={current_position['y']:.2f}, angle={angle:.2f}"
        )

        # 初始化变量
        total_distance = 0
        recorded_points = [{"x": current_position["x"], "y": current_position["y"]}]

        # 开始移动
        while total_distance < 400:
            # 随机前进 2m ~ 6m
            step_distance = random.uniform(2, 6)
            angle = adjust_angle(angle)
            new_position = move_along_angle(
                current_position["x"], current_position["y"], angle, step_distance
            )

            # 检查是否碰到边界
            if is_outside_border(new_position, border) or is_in_obstacle(
                new_position, obstacles
            ):
                continue
                print("碰到边界，调整到边界位置...")
                # 调整位置到边界，记录当前点
                collision_point = find_collision_point(
                    current_position, new_position, border
                )
                recorded_points.append(collision_point)
                current_position = collision_point
                angle = (angle + math.pi) % (2 * math.pi)
                total_distance += math.sqrt(
                    (collision_point["x"] - current_position["x"]) ** 2
                    + (collision_point["y"] - current_position["y"]) ** 2
                )
                continue

            # 添加新点
            current_position = new_position
            recorded_points.append(current_position)
            angle = adjust_angle(angle)
            total_distance += step_distance

        # 更新 poi 列表
        data["poi"] = recorded_points

        # 写回文件
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)

        print(f"路径生成完成，总行走距离：{total_distance:.2f} m")
        print("生成的路径点如下：")
        for i, point in enumerate(recorded_points):
            print(f"点 {i + 1}: x={point['x']:.2f}, y={point['y']:.2f}")

    except FileNotFoundError:
        print(f"错误：文件 {file_path} 未找到！")
    except json.JSONDecodeError:
        print(f"错误：无法解析文件 {file_path} 的 JSON 内容！")
    except KeyError as e:
        print(f"错误：JSON 数据中缺少关键字段 {e}！")


if __name__ == "__main__":
    # 从命令行获取文件名
    if len(sys.argv) < 3:
        print("用法: python script.py <base文件路径> <生成的文件名>")
        sys.exit(1)

    base_path = sys.argv[1]
    file_path = sys.argv[2]
    process_file(base_path, file_path)
