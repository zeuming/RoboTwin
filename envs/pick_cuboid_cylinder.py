
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
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.step_lim = 280
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self):
        self.left_block1, self.left_block1_data = create_obj(
            self.scene,
            pose = sapien.Pose([-0.25,0.26,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.left_block2, self.left_block2_data = create_obj(
            self.scene,
            pose = sapien.Pose([-0.1,0.26,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )

        self.right_block1, self.right_block1_data = create_obj(
            self.scene,
            pose = sapien.Pose([0.1,0.26,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.right_block2, self.right_block2_data = create_obj(
            self.scene,
            pose = sapien.Pose([0.25,0.26,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )

        qpose1 = [[1,0,0,0],[0.707,0.707,0,0]]
        ylim1 = [[0.05,0.2],[0.02,0.05]]
        zlim1 = [[0.82], [0.773]]
        rotate_lim1 = [(0,0,1.57), (0,math.pi/4,0)]
        
        qpose2 = [[0.707,0,0.707,0],[0.5,0.5,0.5,0.5]]
        ylim2 = [[0.05,0.2],[0.02,0.05]]
        zlim2 = [[0.82], [0.773]]
        rotate_lim2 = [(0,0,0), (0,math.pi/4,0)]

        tag = np.random.randint(0,2)
        pose = rand_pose(
            xlim=[-0.25,-0.15],
            ylim=ylim1[tag],
            zlim=zlim1[tag],
            qpos=qpose1[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim1[tag]
        )

        self.cuboid = create_box(
            self.scene,
            pose = pose,
            half_size=(0.0235,0.0235,0.07),
            color = (0,0.5,1),
            name = "cuboid"
        )
        
        # tag = np.random.randint(0,2)
        # # tag = 0
        # pose1 = rand_pose(
        #     xlim=[-0.25,-0.15],
        #     ylim=ylim2[tag],
        #     zlim=zlim2[tag],
        #     qpos=qpose2[tag],
        #     rotate_rand=True,
        #     rotate_lim=rotate_lim2[tag]
        # )

        # while np.sum(pow(pose1.p[:2] - pose.p[:2], 2)) < 0.005:
        #     pose1 = rand_pose(
        #         xlim=[-0.25,-0.15],
        #         ylim=ylim2[tag],
        #         zlim=zlim2[tag],
        #         qpos=qpose2[tag],
        #         rotate_rand=True,
        #         rotate_lim=rotate_lim2[tag]
        #     )

        # self.left_cylinder = create_cylinder(
        #     self.scene,
        #     pose = pose1,
        #     radius=0.0235,
        #     half_length=0.07,
        #     color = (1,0.5,0),
        #     name = "left_cylinder"
        # )



        # tag = np.random.randint(0,2)
        # pose = rand_pose(
        #     xlim=[0.15,0.25],
        #     ylim=ylim1[tag],
        #     zlim=zlim1[tag],
        #     qpos=qpose1[tag],
        #     rotate_rand=True,
        #     rotate_lim=rotate_lim1[tag]
        # )

        # self.right_cuboid = create_box(
        #     self.scene,
        #     pose = pose,
        #     half_size=(0.0235,0.0235,0.07),
        #     color = (0,0.5,1),
        #     name = "right_cuboid"
        # )
        
        tag = np.random.randint(0,2)
        # tag = 0
        pose = rand_pose(
            xlim=[0.15,0.25],
            ylim=ylim2[tag],
            zlim=zlim2[tag],
            qpos=qpose2[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim2[tag]
        )

        # while np.sum(pow(pose.p[:2] - pose.p[:2], 2)) < 0.005:
        #     pose = rand_pose(
        #         xlim=[0.15,0.25],
        #         ylim=ylim2[tag],
        #         zlim=zlim2[tag],
        #         qpos=qpose2[tag],
        #         rotate_rand=True,
        #         rotate_lim=rotate_lim2[tag]
        #     )

        self.cylinder = create_cylinder(
            self.scene,
            pose = pose,
            radius=0.0235,
            half_length=0.07,
            color = (1,0.5,0),
            name = "cylinder"
        )
        # self.left_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.left_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.right_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.right_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def play_once(self):
        # while 1:
        #     self.close_right_gripper()

        cuboid_pose = self.cuboid.get_pose().p
        cuboid_rpy = self.cuboid.get_pose().get_rpy()
        # print(cuboid_pose[2], cuboid_rpy)
        if cuboid_pose[2] > 0.8:
            # pose1
            arg = math.fmod(cuboid_rpy[2], math.pi / 2)
            grasp_arg = arg - math.pi/2  if arg > math.pi/4 else - math.pi/2 + arg
            grasp_quat = t3d.euler.euler2quat(0,0,grasp_arg)
            grasp_qpose1 = t3d.quaternions.qmult(grasp_quat, [0.707,0,0,0.707]).tolist()
            left_pose1 = self.get_grasp_pose_w_given_direction(self.cuboid,grasp_qpos=grasp_qpose1,pre_dis=0.1)
            left_pose2 = self.get_grasp_pose_w_given_direction(self.cuboid,grasp_qpos=grasp_qpose1,pre_dis=0.02)
        else:
            grasp_matrix = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])
            left_pose1 = self.get_grasp_pose_w_labeled_direction(self.cuboid,grasp_matrix=grasp_matrix,pre_dis=0.1)
            left_pose2 = self.get_grasp_pose_w_labeled_direction(self.cuboid,grasp_matrix=grasp_matrix,pre_dis=0.02)


        cylinder_pose = self.cylinder.get_pose().p
        cylinder_rpy = self.cylinder.get_pose().get_rpy()
        # print(cylinder_pose[2], cylinder_rpy)
        if cylinder_pose[2] > 0.8:
            # pose1
            right_pose1 = self.get_grasp_pose_w_given_direction(self.cylinder, grasp_qpos=[0.707,0,0,0.707], pre_dis=0.1)
            right_pose2 = self.get_grasp_pose_w_given_direction(self.cylinder, grasp_qpos=[0.707,0,0,0.707], pre_dis=0.02)
        else:
            right_pose1 = self.get_grasp_pose_w_labeled_direction(self.cylinder,pre_dis=0.1)
            right_pose2 = self.get_grasp_pose_w_labeled_direction(self.cylinder,pre_dis=0.02)

        self.together_move_to_pose_with_screw(left_target_pose=left_pose1,right_target_pose=right_pose1,save_freq = 15)
        self.together_move_to_pose_with_screw(left_target_pose=left_pose2,right_target_pose=right_pose2,save_freq = 15)
        self.together_close_gripper(save_freq = 15)
        left_pose2[2] += 0.08
        right_pose2[2] += 0.08
        left_pose2[1] -= 0.02
        right_pose2[1] -= 0.02
        self.together_move_to_pose_with_screw(left_target_pose=left_pose2,right_target_pose=right_pose2,save_freq = 15)

        left_block1_goal_pose = self.get_actor_goal_pose(self.left_block1,self.left_block1_data) + [0,0,0.12]
        right_block2_goal_pose = self.get_actor_goal_pose(self.right_block2,self.right_block2_data) + [0,0,0.12]

        print(left_block1_goal_pose)
        print(right_block2_goal_pose,'\n')
        left_target_pose = self.get_target_pose_from_goal_point_and_direction(self.cuboid,endpose = self.left_endpose,target_pose = left_block1_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])
        right_target_pose = self.get_target_pose_from_goal_point_and_direction(self.cylinder,endpose = self.right_endpose,target_pose = right_block2_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])

        print(left_target_pose)
        print(right_target_pose)

        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq = 15)
        # left_block1_goal_pose[2] -= 0.05
        # right_block2_goal_pose[2] -= 0.05
        # left_target_pose = self.get_target_pose_from_goal_point_and_direction(self.cuboid,endpose = self.left_endpose,target_pose = left_block1_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])
        # right_target_pose = self.get_target_pose_from_goal_point_and_direction(self.cylinder,endpose = self.right_endpose,target_pose = right_block2_goal_pose, target_grasp_qpose=[0.707,0,0,0.707])

        left_target_pose[2] -= 0.05
        right_target_pose[2] -= 0.05
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq = 15)

        while 1:
            self.close_right_gripper()

    def check_success(self):
        return 1
