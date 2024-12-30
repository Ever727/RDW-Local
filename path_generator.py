import json
import math
import random
import sys

from shapely import Point, Polygon, LineString


def is_illegal_position(point, border, obstacle_list):
    x, y = point["x"], point["y"]
    if not Polygon(border).contains(Point(x, y)):
        return True
    for obstacle in obstacle_list:
        if Polygon(obstacle).contains(Point(x, y)):
            return True
    return False


def random_point_in_polygon(polygon):
    """
    随机生成一个在多边形内的点。
    """
    min_x, min_y, max_x, max_y = Polygon(polygon).bounds
    x = random.uniform(min_x, max_x)
    y = random.uniform(min_y, max_y)
    return x, y


def move_along_angle(x, y, angle, distance):
    """
    根据当前位置、角度和移动距离，计算新位置。
    """
    new_x = x + distance * math.cos(angle)
    new_y = y + distance * math.sin(angle)
    return {"x": new_x, "y": new_y}


def is_intersect(point1, point2, obstacle_list):
    x1, y1 = point1["x"], point1["y"]
    x2, y2 = point2["x"], point2["y"]
    for obstacle in obstacle_list:
        if LineString([(x1, y1), (x2, y2)]).intersects(Polygon(obstacle)):
            return True
    return False


def adjust_angle():
    """
    随机调整角度，范围 0 到 2*pi。
    """
    return random.uniform(0, 2 * math.pi)


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
        border = [(t["x"], t["y"]) for t in data["border_virt"]]
        raw_obstacle_list = data["obstacles_virt"]
        obstacles = []
        for raw_obstacle in raw_obstacle_list:
            obstacle = [(t["x"], t["y"]) for t in raw_obstacle]
            obstacles.append(obstacle)

        # 现实环境
        current_position_phys = data["initial_user_phys"]
        border_phys = [(t["x"], t["y"]) for t in data["border_phys"]]
        raw_obstacle_list = data["obstacles_phys"]
        obstacles_phys = []
        for raw_obstacle in raw_obstacle_list:
            obstacle = [(t["x"], t["y"]) for t in raw_obstacle]
            obstacles_phys.append(obstacle)

        # 随机化初始位置
        current_position_phys["x"], current_position_phys["y"] = (
            random_point_in_polygon(border_phys)
        )
        while is_illegal_position(current_position_phys, border_phys, obstacles_phys):
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
        while is_illegal_position(current_position, border, obstacles):
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

        # 统一虚拟和物理点
        is_initial = False
        while not is_initial:
            # 随机前进 2m ~ 6m
            step_distance = random.uniform(2, 6)
            angle = adjust_angle()
            new_position = move_along_angle(
                current_position["x"], current_position["y"], angle, step_distance
            )

            # 检查是否碰到边界
            if is_illegal_position(new_position, border, obstacles) or is_intersect(
                current_position, new_position, obstacles
            ):
                continue

            # 初始化点
            current_position["angle"] = angle
            data["initial_user_virt"] = current_position

            # 如果不需要统一物理和虚拟点，则注释掉下面一行
            data["initial_user_phys"] = current_position

            # 添加新点
            current_position = new_position
            recorded_points.append(current_position)
            angle = adjust_angle()
            total_distance += step_distance
            is_initial = True

        # 开始移动
        while total_distance < 50:
            # 随机前进 2m ~ 6m
            step_distance = random.uniform(2, 6)
            angle = adjust_angle()
            new_position = move_along_angle(
                current_position["x"], current_position["y"], angle, step_distance
            )

            # 检查是否碰到边界
            if is_illegal_position(new_position, border, obstacles) or is_intersect(
                current_position, new_position, obstacles
            ):
                continue

            # 添加新点
            current_position = new_position
            recorded_points.append(current_position)
            angle = adjust_angle()
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
