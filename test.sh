nums=25
algorithms=("ARC" "NON" "S2C" "S2O" "APF" "RL")  # 定义算法列表

# Function to handle cleanup on Ctrl+C
cleanup() {
  echo "Cleaning up background processes..."
  kill $client_pid 2>/dev/null
  kill $uvicorn_pid 2>/dev/null
  kill $python_pid 2>/dev/null
  pkill geckodriver
}

# Trap Ctrl+C (SIGINT) to trigger the cleanup function
trap "cleanup; exit 1" SIGINT

# 检查 type=end 的函数
check_end_condition() {
  log_file=$1
  while true; do
    # 读取最后一行
    last_line=$(tail -n 2 "$log_file" 2>/dev/null)
    
    # 检查 JSON 的 type 字段
    if echo "$last_line" | grep -q "'type': 'end'"; then
      echo "检测到 'type=end'，继续执行下一步..."
      break
    fi

    sleep 0.1  # 每秒检查一次
  done
}

# 遍历 environment 从 2 到 10
for environment in {9..10}; do
  echo "正在测试环境 $environment"
  
  # 遍历 algorithms
  for algorithm in "${algorithms[@]}"; do
    echo "正在测试算法 $algorithm"
    times=1  # 每次切换算法时重置 times
    while [ $times -le $nums ]; do
      echo "========== 开始第 $times 次执行 (环境: $environment, 算法: $algorithm) =========="

      # 检查端口占用并清理
      pid=$(lsof -t -i :8000)
      if [ -n "$pid" ]; then
        kill -9 $pid
        echo "已终止占用端口 8000 的进程 (PID: $pid)"
      fi

      pid=$(lsof -t -i :8765)
      if [ -n "$pid" ]; then
        kill -9 $pid
        echo "已终止占用端口 8765 的进程 (PID: $pid)"
      fi

      # 启动 client_base.py
      log_file="logs/env_${environment}/${times}_${algorithm}.txt"
      mkdir -p "logs/env_${environment}"  # 确保日志目录存在
      mkdir -p "graphs/env_${environment}"  # 确保日志目录存在
      bash -c "python client_base.py -c ${algorithm} > $log_file" &
      client_pid=$!
      echo "后端进程 PID: $client_pid"

      # 启动 uvicorn
      bash -c "cd ../RDW-Web && uvicorn main:app --host 0.0.0.0 --port 8000 > /dev/null 2>&1" &
      uvicorn_pid=$!
      echo "前端进程 PID: $uvicorn_pid"

      sleep 0.1

      # 启动 firefox.py
      bash -c "python firefox.py 'test_set/env_${environment}/${times}.json'" &
      python_pid=$!
      echo "浏览器进程 PID: $python_pid"

      # 检查日志文件是否包含 type=end
      echo "正在等待日志文件中 'type=end' 的出现..."
      check_end_condition "$log_file"

      # 运行 draw.py
      python draw.py "$log_file"
      python draw_virtual.py "$log_file"

      # 清理后台进程
      echo "Cleaning up background processes..."
      kill $client_pid 2>/dev/null
      kill $uvicorn_pid 2>/dev/null
      kill $python_pid 2>/dev/null
      pkill geckodriver
      echo "Finish cleaning up"

      # 增加 times
      times=$((times + 1))

      echo "========== 第 $((times - 1)) 次执行完成 (环境: $environment, 算法: $algorithm) =========="
    done
  done
  echo "绘制柱状图和箱线图"
  python draw_reset_bar.py "logs/env_${environment}/"
  python draw_reset_box.py "logs/env_${environment}/"

done

echo "所有环境和算法的任务执行完成！"
exit 0
