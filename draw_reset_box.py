import os
import ast
import sys
import matplotlib.pyplot as plt

def parse_log_file(file_path):
    """
    解析日志文件，返回reset计数
    """
    reset_count = 0
    print("file_path: ", file_path)
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith("{'type':"):
                try:
                    # 将每一行转换为字典
                    message = ast.literal_eval(line.strip())
                    if message['type'] == 'running':
                        if 'need_reset' in message and message['need_reset']:
                            reset_count += 1
                        if 'reset' in message and message['reset']:
                            reset_count += 1
                except Exception as e:
                    print(f"Error parsing line: {line}, Error: {e}")
    return reset_count

def collect_reset_data(path, algorithms):
    """
    收集每个算法的reset次数数据（而不是平均值）
    """
    reset_data = {alg: [] for alg in algorithms}
    
    for alg in algorithms:
        for i in range(1, 26):  # 每个算法有25次测量
            log_file = os.path.join(path, f"{i}_{alg}.txt")
            if os.path.exists(log_file):
                reset_count = parse_log_file(log_file)
                if reset_count < 200:
                    reset_data[alg].append(reset_count)
            else:
                print(f"Warning: File {log_file} does not exist.")
    
    return reset_data

def plot_boxplot(reset_data, save_path):
    """
    绘制箱线图，显示每个算法的reset次数分布，并为每个箱体填充不同颜色
    """
    algorithms = list(reset_data.keys())
    data = list(reset_data.values())

    # 为每个算法的箱线图指定不同的颜色
    colors = ['skyblue', 'lightgreen', 'lightcoral', 'plum', 'lightpink', 'lightgray']

    # 绘制箱线图
    boxprops = dict(linewidth=1, color='black')
    medianprops = dict(color='red', linewidth=2)
    flierprops = dict(marker='o', color='orange', markersize=6)

    # 绘制箱线图，并为每个箱体填充颜色
    boxes = plt.boxplot(data, labels=algorithms, patch_artist=True, 
                        boxprops=boxprops, medianprops=medianprops, 
                        flierprops=flierprops)

    # 为每个算法的箱线图设置不同的填充颜色
    for i, box in enumerate(boxes['boxes']):
        box.set_facecolor(colors[i])  # 设置每个箱体的填充颜色

    plt.xlabel('Algorithm')
    plt.ylabel('Reset Count')
    plt.title('Reset Count Distribution for Each Algorithm')
    plt.xticks(rotation=45)
    plt.tight_layout()

    # 保存图表到文件
    plt.savefig(save_path)

def main():
    # 获取用户输入的path
    path = sys.argv[1]
    
    # 算法列表
    algorithms = ['S2C', 'S2O', 'APF', 'ARC', 'RL', 'NON']
    
    # 收集每个算法的reset次数数据
    reset_data = collect_reset_data(path, algorithms)
    
    save_path = "reset_counts/" + path[5:10] + "_box" + ".png"
    
    # 绘制箱线图
    plot_boxplot(reset_data, save_path)

if __name__ == "__main__":
    main()
