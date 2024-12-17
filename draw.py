import matplotlib.pyplot as plt
import matplotlib.patches as patches
import ast
import sys

# 读取文件内容
if len(sys.argv) < 2:
    print("用法: python draw.py <文件路径>")
    sys.exit(1)

file_path = sys.argv[1]  # 获取文件名
data = ""

with open(file_path, 'r', encoding='utf-8') as file:
    data = file.read()
file.close()
file_path = file_path[5:-4]  

# 将数据拆分成独立的消息
lines = data.strip().split('\n')

# 初始化变量
border = []
obstacles = []
user_positions = []

for line in lines:
    # 将字符串形式的字典转换为实际的字典对象
    message = ast.literal_eval(line)
    
    if message['type'] == 'start':
        # 获取环境尺寸和障碍物信息
        physical_env = message['physical']
        border = physical_env['border']
        obstacles = physical_env['obstacle_list']
    elif message['type'] == 'running':
        # 获取用户位置
        physical_state = message['physical']
        user_x = physical_state['user_x']
        user_y = physical_state['user_y']
        user_positions.append((user_x, user_y))

# 开始绘图
fig, ax = plt.subplots()

# 绘制环境边界
border_x = [point['x'] for point in border] + [border[0]['x']]
border_y = [point['y'] for point in border] + [border[0]['y']]
ax.plot(border_x, border_y, 'k-', linewidth=2)  # 黑色线条表示边界

# 绘制障碍物
for obstacle in obstacles:
    obs_x = [point['x'] for point in obstacle] + [obstacle[0]['x']]
    obs_y = [point['y'] for point in obstacle] + [obstacle[0]['y']]
    ax.fill(obs_x, obs_y, 'red')  # 灰色填充的障碍物

# 绘制用户行动路线
if user_positions:
    user_xs, user_ys = zip(*user_positions)
    ax.plot(user_xs, user_ys, 'k-', linewidth=0.8, label='User Path')  # 红色线连接用户位置

# 设置坐标轴范围
ax.set_xlim(0, physical_env['width'])
ax.set_ylim(0, physical_env['height'])

# 设置等比例显示
ax.set_aspect('equal', 'box')

# 添加标签和标题
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('User Path ' + file_path)
ax.legend()

# 显示图形
plt.savefig('graphs/' + file_path + '.png')
