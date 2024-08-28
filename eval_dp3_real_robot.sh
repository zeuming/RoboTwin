# bash script/eval_dp3.sh robot_dp3 ${task name} eval ${seed} ${gpu_id} 
task_name="mug_hanging_50"
seed=0
gpu_id=0
bash script/eval_dp3_real_robot.sh robot_dp3 $task_name eval $seed $gpu_id