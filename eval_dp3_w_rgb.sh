# bash script/eval_dp3.sh robot_dp3 ${task name} eval ${seed} ${gpu_id} 
# bash eval_dp3.sh shoe_place 100 0
task_name=${1}
expert_data_num=${2}
seed=0
gpu_id=${3}
checkpoint_num=3000

bash script/eval_dp3.sh robot_dp3_w_rgb $task_name $expert_data_num eval $seed $checkpoint_num $gpu_id