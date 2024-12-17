times=1
generate_num=25
env_path=test_set/env_1

for i in $(seq 1 $generate_num); do
    bash -c "python path_generator.py ${env_path}/base.json ${env_path}/${times}.json" &
    pid=$!
    wait $pid
    times=$((times + 1))
done