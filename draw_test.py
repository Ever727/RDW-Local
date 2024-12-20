import matplotlib.pyplot as plt
import json
from matplotlib.patches import Polygon
import numpy as np
import sys

# 读取文件内容
if len(sys.argv) < 2:
    print("用法: python draw.py <文件路径>")
    sys.exit(1)

file_path = sys.argv[1]  # 获取文件名

# 读取 JSON 文件
with open(file_path, 'r') as f:
    data = json.load(f)

# 提取数据
border_virt = data['border_virt']
obstacles_virt = data['obstacles_virt']
poi = data['poi']

# 创建绘图
fig, ax = plt.subplots(figsize=(8, 8))

# 绘制边界 (border_virt)
border_x = [point['x'] for point in border_virt]
border_y = [point['y'] for point in border_virt]
border_x.append(border_x[0])  # 闭合多边形
border_y.append(border_y[0])

ax.plot(border_x, border_y, 'b-', label='Boundary')

# 绘制障碍物 (obstacles_virt)
for obstacle in obstacles_virt:
    obstacle_x = [point['x'] for point in obstacle]
    obstacle_y = [point['y'] for point in obstacle]
    obstacle_x.append(obstacle_x[0])  # 闭合多边形
    obstacle_y.append(obstacle_y[0])

    # 绘制障碍物轮廓
    ax.plot(obstacle_x, obstacle_y, 'g-', alpha=0.7)
    
    obs_x = [point['x'] for point in obstacle] + [obstacle[0]['x']]
    obs_y = [point['y'] for point in obstacle] + [obstacle[0]['y']]
    ax.fill(obs_x, obs_y, 'red')

    # 计算障碍物中心
    center_x = np.mean(obstacle_x)
    center_y = np.mean(obstacle_y)

    # 绘制障碍物中心
    # ax.scatter(center_x, center_y, color='red', zorder=5)

# 绘制路径点 (poi)
poi_x = [point['x'] for point in poi]
poi_y = [point['y'] for point in poi]
ax.scatter(poi_x, poi_y, color='purple', label='Path Points')

# 设置图形标签
ax.set_title("Boundary, Obstacles, and Path Points")
ax.set_xlabel("X")
ax.set_ylabel("Y")
# ax.set_xlim(-5, 65)  # 根据实际需要调整
# ax.set_ylim(-5, 65)  # 根据实际需要调整
ax.set_aspect('equal', 'box')
ax.grid(True)

# 显示图例
ax.legend()

# 显示图形
plt.savefig(file_path[:-5] + '.png')
