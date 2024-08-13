import imageio
import os

def create_video(image_folder, output_video_file, save_dir='./task_video/', fps=30):
    # 修复创建目录的语法错误
    video_dir = save_dir
    if not os.path.exists(video_dir):
        os.makedirs(video_dir)

    # 获取并排序图像文件
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images.sort(key=lambda x: int(x.split('.')[0]))

    # 创建视频写入器
    writer = imageio.get_writer(os.path.join(video_dir, output_video_file), fps=fps)

    # 添加图像到视频
    for i, image_name in enumerate(images, start=1):
        img_path = os.path.join(image_folder, image_name)
        try:
            image = imageio.imread(img_path)
            writer.append_data(image)
            print(f"Added {image_name} to video ({i}/{len(images)})", end='\r')
        except Exception as e:
            print(f"Error processing {image_name}: {e}")

    writer.close()
    print("\nVideo creation complete.")

# # 使用函数
# def main():
#     create_video('./policy_data/test/episode0/camera/color/front/0.png', 'pick_cup_with_liquid_top.mp4')

if __name__ == '__main__':
    task_list = ['hammer_beat', 'move_bottle', 'open_cabinet_put_apple', 'pick_bottles', 'pick_empty_cup', 'put_block_into_dustpan']
    load_dir = './data'
    # for task in task_list:
        # create_video(f'{load_dir}/{task}/episode0/camera/color/front/',f'{task}_top.mp4', save_dir=f'./task_video/success/')
        # create_video(f'{load_dir}/{task}/episode0/camera/color/expert/',f'{task}_expert.mp4', save_dir='./task_video/success/')
        # create_video(f'{load_dir}/{task}/episode1/camera/color/front/',f'{task}_top.mp4', save_dir=f'./task_video/fail/')
        # create_video(f'{load_dir}/{task}/episode1/camera/color/expert/',f'{task}_expert.mp4', save_dir='./task_video/fail/')

        # create_video(f'{load_dir}/{task}/episode0/camera/color/front/',f'{task}_top.mp4', save_dir=f'./task_video_benchmark/')
        # create_video(f'{load_dir}/{task}/episode0/camera/color/expert/',f'{task}_expert.mp4', save_dir='./task_video_benchmark/')
