times=1
environment=2
nums=2

while [ $times -le $nums ]; do

  # 运行 draw.py
  log_file="logs/env_${environment}/${times}_S2C.txt"
  python draw_virtual.py "$log_file"

  # 增加 times
  times=$((times + 1))

  echo "========== 第 $((times - 1)) 次执行完成 (算法) =========="
done

echo "所有算法和任务执行完成！"
exit 0
