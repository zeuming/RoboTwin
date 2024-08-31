
from .base_task import Base_task
from .utils import *
import math
import sapien

# ing!
class pick_cuboid_cylinder(Base_task):
    def setup_demo(self,is_test = False, **kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
        self.step_lim = 280
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self):
        left_pose_que = np.random.permutation([sapien.Pose([-0.25,0.2,0.77] , [0.707,0.707,0,0]), sapien.Pose([-0.1,0.2,0.77] , [0.707,0.707,0,0])])
        right_pose_que = np.random.permutation([sapien.Pose([0.1,0.2,0.77] , [0.707,0.707,0,0]), sapien.Pose([0.25,0.2,0.77] , [0.707,0.707,0,0])])

        self.left_block1, self.left_block1_data = create_obj(
            self.scene,
            pose = left_pose_que[0],
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.left_block2, self.left_block2_data = create_obj(
            self.scene,
            pose = left_pose_que[1],
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )

        self.right_block1, self.right_block1_data = create_obj(
            self.scene,
            pose = right_pose_que[0],
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.right_block2, self.right_block2_data = create_obj(
            self.scene,
            pose = right_pose_que[1],
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )

        qpose1 = [[1,0,0,0],[0.707,0.707,0,0]]
        ylim1 = [[-0.1,0.2],[-0.23,0.03]]
        zlim1 = [[0.82], [0.773]]
        rotate_lim1 = [(0,0,1.57), (0,math.pi/4,0)]
        
        qpose2 = [[0.707,0,0.707,0],[0.5,0.5,0.5,0.5]]
        ylim2 = [[-0.1,0.2],[-0.23,0.03]]
        zlim2 = [[0.82], [0.773]]
        rotate_lim2 = [(0,0,0), (0,math.pi/4,0)]

        ############################left cuboid and cylinder######################################
        tag = np.random.randint(0,2)
        tag=1
        pose = rand_pose(
            xlim=[-0.25,-0.1],
            ylim=ylim1[tag],
            zlim=zlim1[tag],
            qpos=qpose1[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim1[tag]
        )

        self.left_cuboid = create_box(
            self.scene,
            pose = pose,
            half_size=(0.0235,0.0235,0.05),
            color = (0,0.5,1),
            name = "cuboid"
        )
        
        tag = np.random.randint(0,2)
        tag=1
        pose1 = rand_pose(
            xlim=[-0.25,-0.1],
            ylim=ylim2[tag],
            zlim=zlim2[tag],
            qpos=qpose2[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim2[tag]
        )

        cnt = 0
        while np.sum(pow(pose.p - pose1.p,2)) < 0.0169 and cnt < 10:
            pose1 = rand_pose(
                xlim=[-0.25,-0.1],
                ylim=ylim2[tag],
                zlim=zlim2[tag],
                qpos=qpose2[tag],
                rotate_rand=True,
                rotate_lim=rotate_lim2[tag]
            )
            cnt += 1
        
        self.left_cylinder = create_cylinder(
            self.scene,
            pose = pose1,
            radius=0.0235,
            half_length=0.05,
            color = (1,0.5,0),
            name = "cylinder"
        )

        ###############################right cuboid and cylinder#####################################
        tag = np.random.randint(0,2)
        tag=1
        pose = rand_pose(
            xlim=[0.1,0.25],
            ylim=ylim1[tag],
            zlim=zlim1[tag],
            qpos=qpose1[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim1[tag]
        )

        self.right_cuboid = create_box(
            self.scene,
            pose = pose,
            half_size=(0.0235,0.0235,0.05),
            color = (0,0.5,1),
            name = "cuboid"
        )
        
        tag = np.random.randint(0,2)
        tag=1
        pose1 = rand_pose(
            xlim=[0.1,0.25],
            ylim=ylim2[tag],
            zlim=zlim2[tag],
            qpos=qpose2[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim2[tag]
        )

        cnt = 0
        while np.sum(pow(pose.p - pose1.p,2)) < 0.0169 and cnt < 10:
            pose1 = rand_pose(
                xlim=[0.1,0.25],
                ylim=ylim2[tag],
                zlim=zlim2[tag],
                qpos=qpose2[tag],
                rotate_rand=True,
                rotate_lim=rotate_lim2[tag]
            )
            cnt += 1

        self.right_cylinder = create_cylinder(
            self.scene,
            pose = pose1,
            radius=0.0235,
            half_length=0.05,
            color = (1,0.5,0),
            name = "cylinder"
        )
        self.left_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.left_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.right_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.right_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def get_grasp_pose_from_cubiod_cylinder(self,actor):
        name = actor.get_name()
        if name == 'cylinder':
            cylinder_pose = actor.get_pose().p
            if cylinder_pose[2] > 0.8:
                # pose1
                pose1 = self.get_grasp_pose_w_given_direction(actor, grasp_qpos=[0.707,0,0,0.707], pre_dis=0.1)
                pose2 = self.get_grasp_pose_w_given_direction(actor, grasp_qpos=[0.707,0,0,0.707], pre_dis=0.02)
            else:
                pose1 = self.get_grasp_pose_w_labeled_direction(actor,pre_dis=0.1)
                pose2 = self.get_grasp_pose_w_labeled_direction(actor,pre_dis=0.02)
        else:
            cuboid_pose = actor.get_pose().p
            cuboid_rpy = actor.get_pose().get_rpy()
            if cuboid_pose[2] > 0.8:
                # pose1
                arg = math.fmod(cuboid_rpy[2], math.pi / 2)
                grasp_arg = arg - math.pi/2  if arg > math.pi/4 else - math.pi/2 + arg
                grasp_quat = t3d.euler.euler2quat(0,0,grasp_arg)
                grasp_qpose1 = t3d.quaternions.qmult(grasp_quat, [0.707,0,0,0.707]).tolist()
                pose1 = self.get_grasp_pose_w_given_direction(actor,grasp_qpos=grasp_qpose1,pre_dis=0.1)
                pose2 = self.get_grasp_pose_w_given_direction(actor,grasp_qpos=grasp_qpose1,pre_dis=0.02)
            else:
                grasp_matrix = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])
                pose1 = self.get_grasp_pose_w_labeled_direction(actor,grasp_matrix=grasp_matrix,pre_dis=0.1)
                pose2 = self.get_grasp_pose_w_labeled_direction(actor,grasp_matrix=grasp_matrix,pre_dis=0.02)
        return pose1,pose2

    def pick_cuboid_and_cylinder(self,left_actor,right_actor,left_block,right_block,left_block_data,right_block_data):

        left_pose1, left_pose2 = self.get_grasp_pose_from_cubiod_cylinder(left_actor)
        right_pose1, right_pose2 = self.get_grasp_pose_from_cubiod_cylinder(right_actor)

        self.together_move_to_pose_with_screw(left_target_pose=left_pose1,right_target_pose=right_pose1,save_freq = 15)
        self.together_move_to_pose_with_screw(left_target_pose=left_pose2,right_target_pose=right_pose2,save_freq = 15)
        self.together_close_gripper(save_freq = 15)
        left_pose2[2] += 0.1
        right_pose2[2] += 0.1
        self.together_move_to_pose_with_screw(left_target_pose=left_pose2,right_target_pose=right_pose2,save_freq = 15)

        left_block_goal_pose = self.get_actor_goal_pose(left_block,left_block_data) + [0,0,0.14]
        right_block_goal_pose = self.get_actor_goal_pose(right_block,right_block_data) + [0,0,0.14]

        left_target_pose = self.get_target_pose_from_goal_point_and_direction(left_actor,endpose = self.left_endpose,target_pose = left_block_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])
        right_target_pose = self.get_target_pose_from_goal_point_and_direction(right_actor,endpose = self.right_endpose,target_pose = right_block_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])

        left_target_pose[1] -= 0.01
        left_target_pose[0] += 0.008
        right_target_pose[1] -= 0.01
        right_target_pose[0] += 0.017

        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq = 15)
        left_target_pose[2] -= 0.11
        right_target_pose[2] -= 0.11
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq = 15)
        self.together_open_gripper(save_freq = 15)
        left_target_pose[2] += 0.09
        right_target_pose[2] += 0.09
        left_target_pose[1] -= 0.05
        right_target_pose[1] -= 0.05
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq = 15)

    def play_once(self):

        if self.left_cuboid.get_pose().p[1] > self.left_cylinder.get_pose().p[1]:
            left_que = [self.left_cuboid, self.left_cylinder]
            left_block_que = [self.left_block1, self.left_block2]
            left_block_data_que = [self.left_block1_data, self.left_block2_data]
        else:
            left_que = [self.left_cylinder, self.left_cuboid]
            left_block_que = [self.left_block2, self.left_block1]
            left_block_data_que = [self.left_block2_data, self.left_block1_data]
        
        if self.right_cuboid.get_pose().p[1] > self.right_cylinder.get_pose().p[1]:
            right_que = [self.right_cuboid, self.right_cylinder] 
            right_block_que = [self.right_block1, self.right_block2]
            right_block_data_que = [self.right_block1_data, self.right_block2_data]
        else:
            right_que = [self.right_cylinder, self.right_cuboid]
            right_block_que = [self.right_block2, self.right_block1]
            right_block_data_que = [self.right_block2_data, self.right_block1_data]

        self.pick_cuboid_and_cylinder(left_que[0],right_que[0],left_block_que[0],right_block_que[0],left_block_data_que[0],right_block_data_que[0])
        self.pick_cuboid_and_cylinder(left_que[1],right_que[1],left_block_que[1],right_block_que[1],left_block_data_que[1],right_block_data_que[1])

    def check_success(self):
        left_cuboid_pose = self.left_cuboid.get_pose().p
        right_cuboid_pose = self.right_cuboid.get_pose().p
        left_cylinder_pose = self.left_cylinder.get_pose().p
        right_cylinder_pose = self.right_cylinder.get_pose().p

        left_block1_pose = self.left_block1.get_pose().p
        right_block1_pose = self.right_block1.get_pose().p
        left_block2_pose = self.left_block2.get_pose().p
        right_block2_pose = self.right_block2.get_pose().p

        eps = [0.015,0.015,0.01]
        return np.all(abs(left_cuboid_pose - left_block1_pose - [0,0,0.045]) < eps) and\
               np.all(abs(right_cuboid_pose - right_block1_pose - [0,0,0.045]) < eps) and\
               np.all(abs(left_cylinder_pose - left_block2_pose - [0,0,0.045]) < eps) and\
               np.all(abs(right_cylinder_pose - right_block2_pose - [0,0,0.045]) < eps) and self.is_left_gripper_open() and self.is_right_gripper_open()
