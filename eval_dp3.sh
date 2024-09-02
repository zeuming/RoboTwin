# bash script/eval_dp3.sh robot_dp3 ${task name} eval ${seed} ${gpu_id} 
task_name="block_hammer_beat"
expert_data_num=50
seed=0
gpu_id=0
checkpoint_num=3000

bash script/eval_dp3.sh robot_dp3 $task_name $expert_data_num eval $seed $checkpoint_num $gpu_id