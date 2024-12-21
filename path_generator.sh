times=1
generate_num=25
environment=8
env_path="test_set/env_${environment}"

mkdir -p "test_set/env_${environment}"  # 确保日志目录存在
# python environment_generator.py "test_set/env_${environment}/base.json"


for i in $(seq 1 $generate_num); do
    bash -c "python path_generator.py ${env_path}/base.json ${env_path}/${times}.json" &
    pid=$!
    wait $pid
    times=$((times + 1))
done