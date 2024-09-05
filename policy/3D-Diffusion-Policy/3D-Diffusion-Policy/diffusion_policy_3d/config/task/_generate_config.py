import os

# 原始内容模板
content_template = """name: {name}

shape_meta: &shape_meta
  # acceptable types: rgb, low_dim
  obs:
    point_cloud:
      shape: [1024, 6]
      type: point_cloud
    agent_pos:
      shape: [14]
      type: low_dim
  action:
    shape: [14]

env_runner:
  _target_: diffusion_policy_3d.env_runner.robot_runner.RobotRunner
  max_steps: 300
  n_obs_steps: ${{n_obs_steps}}
  n_action_steps: ${{n_action_steps}}
  task_name: robot

dataset:
  _target_: diffusion_policy_3d.dataset.robot_dataset.RobotDataset
  zarr_path: data/{name}.zarr
  horizon: ${{horizon}}
  pad_before: ${{eval:'${{n_obs_steps}}-1'}}
  pad_after: ${{eval:'${{n_action_steps}}-1'}}
  seed: 0
  val_ratio: 0.02
  max_train_episodes: null
"""

# 要生成的文件的名称列表
names = names = [
    "block_hammer_beat",
    "blocks_stack_easy",
    "dual_bottles_pick_hard",
    "pick_apple_messy",
    "block_handover",
    "container_place",
    "empty_cup_place",
    "pick_cuboid_cylinder",
    "apple_cabinet_storage",
    "block_sweep",
    "diverse_bottles_pick",
    "mug_hanging",
    "shoe_place",
    "blocks_stack",
    "dual_bottles_pick",
    "multi_object_storage",
    "shoes_place"
]

# 指定输出目录
output_dir = "./"

# 确保输出目录存在
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 为每个名称生成文件
for orig_name in names:
    num = ['20', '50', '100']
    for x in num:
        name = orig_name + '_' + x
        # 替换模板中的{name}占位符
        content = content_template.format(name=name)
        # 定义文件路径
        file_path = os.path.join(output_dir, f"{name}.yaml")
        # 写入文件
        with open(file_path, 'w') as file:
            file.write(content)
        print(f"文件已生成: {file_path}")

print("所有文件已生成完毕。")