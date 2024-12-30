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

def calculate_avg_reset(path, algorithms):
    """
    计算每个算法的平均reset次数
    """
    reset_counts = {alg: 0.0 for alg in algorithms}
    
    for alg in algorithms:
        total_reset_count = 0
        times = 0
        for i in range(1, 26):  # 每个算法有25次测量
            log_file = os.path.join(path, f"{i}_{alg}.txt")
            if os.path.exists(log_file):
                reset_count = parse_log_file(log_file)
                if reset_count < 200:
                    total_reset_count += reset_count
                    times += 1
            else:
                print(f"Warning: File {log_file} does not exist.")
        
        # 计算平均reset次数
        avg_reset_count = total_reset_count / times
        reset_counts[alg] = avg_reset_count
    
    return reset_counts

def plot_bar_chart(reset_counts, save_path):
    """
    绘制每个算法的平均reset次数的柱状图，并为每个柱填充不同的颜色
    """
    algorithms = list(reset_counts.keys())
    avg_resets = list(reset_counts.values())

    # 为每个柱子选择一个不同的颜色
    colors = ['lightblue', 'lightgreen', 'lightcoral', 'plum', 'lightpink', 'lightgray']
    
    # 确保颜色数量与算法数量一致
    if len(colors) < len(algorithms):
        colors = colors * (len(algorithms) // len(colors)) + colors[:len(algorithms) % len(colors)]

    # 绘制柱状图
    bars = plt.bar(algorithms, avg_resets, color=colors)
    plt.xlabel('Algorithm')
    plt.ylabel('Average Reset Count')
    plt.title('Average Reset Count for Each Algorithm')
    plt.xticks(rotation=45)
    plt.tight_layout()
    
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width() / 2, height + 0.05, f'{height:.2f}', 
                 ha='center', va='bottom', fontsize=10)
    
    # 保存图表到文件
    plt.savefig(save_path)

def main():
    # 获取用户输入的path
    path = sys.argv[1]
    
    # 算法列表
    algorithms = ['S2C', 'S2O', 'APF', 'ARC', 'RL', 'NON']
    
    # 计算每个算法的平均reset次数
    reset_counts = calculate_avg_reset(path, algorithms)
    
    # 打印每个算法的平均reset次数
    for alg, avg_reset in reset_counts.items():
        print(f"算法 {alg} 的平均 reset 次数: {avg_reset:.2f}")
    
    save_path = "reset_counts/" + path[5:10] + "_bar" + ".png"
    
    # 绘制柱状图
    plot_bar_chart(reset_counts, save_path)

if __name__ == "__main__":
    main()
