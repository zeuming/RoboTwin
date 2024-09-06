# bash train.sh shoe_place 20 0
task_name=${1}
expert_data_num=${2}
seed=0
gpu_id=${3}

bash scripts/train_policy.sh robot_dp3_w_rgb ${task_name}_${expert_data_num} train_w_rgb ${seed} ${gpu_id}