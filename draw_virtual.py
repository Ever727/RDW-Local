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

file_path = file_path[5:-8]  + '_virtual'

# 将数据拆分成独立的消息
lines = data.strip().split('\n')

# 过滤掉不符合格式的行
lines = [line for line in lines if line.startswith("{'type':")]

# 初始化变量
border = []
obstacles = []
user_positions = []
reset_count = 0  # 统计 need_reset 为 True 的次数

for line in lines:
    # 将字符串形式的字典转换为实际的字典对象
    message = ast.literal_eval(line)
    
    if message['type'] == 'start':
        # 获取环境尺寸和障碍物信息
        virtual_env = message['virtual']
        border = virtual_env['border']
        obstacles = virtual_env['obstacle_list']
    elif message['type'] == 'running':
        # 检查 need_reset 是否为 True
        if 'need_reset' in message and message['need_reset']:
            reset_count += 1
        
        # 获取用户位置
        virtual_state = message['virtual']
        user_x = virtual_state['user_x']
        user_y = virtual_state['user_y']
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
    ax.fill(obs_x, obs_y, 'red')  # 红色填充的障碍物

# 绘制用户行动路线
if user_positions:
    user_xs, user_ys = zip(*user_positions)
    ax.plot(user_xs, user_ys, 'k-', linewidth=0.8, label='User Path')  # 黑色线连接用户位置

# 在图中显示 reset 次数
reset_text = f"Reset Count: {reset_count}"

# 设置坐标轴范围
min_x = min(point['x'] for point in border)
max_x = max(point['x'] for point in border)
min_y = min(point['y'] for point in border)
max_y = max(point['y'] for point in border)
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)

# 设置等比例显示
ax.set_aspect('equal', 'box')

# 添加标签和标题
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('User Path ' + file_path)
ax.legend()

# 保存图形
plt.savefig('graphs/' + file_path + '.png')
print(f"图形保存至: graphs/{file_path}.png")
