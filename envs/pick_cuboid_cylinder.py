
from .base_task import Base_task
from .utils import *
import math
import sapien

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
        self.left_block1, self.block1_data = create_obj(
            self.scene,
            pose = sapien.Pose([-0.25,-0.1,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.left_block2, self.block2_data = create_obj(
            self.scene,
            pose = sapien.Pose([-0.1,-0.1,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )


        self.right_block1, self.block1_data = create_obj(
            self.scene,
            pose = sapien.Pose([0.25,-0.1,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=0,
        )
        self.right_block2, self.block2_data = create_obj(
            self.scene,
            pose = sapien.Pose([0.1,-0.1,0.77] , [0.707,0.707,0,0]),
            modelname="004_fluted_block",
            convex=False,
            is_static=True,
            model_id=1,
        )

        qpose1 = [[1,0,0,0],[0.707,-0.707,0,0]]
        zlim1 = [[0.8], [0.765]]
        rotate_lim1 = [(0,0,1.57), (0,0,1.57)]
        
        qpose2 = [[0.707,0,0.707,0],[1,0,0,0]]
        zlim2 = [[0.8], [0.765]]
        rotate_lim2 = [(1.57,0,0), (0,1.57,0)]

        tag = np.random.randint(0,2)
        pose = rand_pose(
            xlim=[-0.25,-0.15],
            ylim=[0.02,0.22],
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
            name = "left_cuboid"
        )
        
        # tag = np.random.randint(0,2)
        tag = 0
        pose1 = rand_pose(
            xlim=[-0.25,-0.15],
            ylim=[0.,0.25],
            zlim=zlim2[tag],
            qpos=qpose2[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim2[tag]
        )

        while np.sum(pow(pose1.p[:2] - pose.p[:2], 2)) < 0.005:
            pose1 = rand_pose(
                xlim=[-0.25,-0.15],
                ylim=[0.,0.25],
                zlim=zlim2[tag],
                qpos=qpose2[tag],
                rotate_rand=True,
                rotate_lim=rotate_lim2[tag]
            )

        self.left_cylinder = create_cylinder(
            self.scene,
            pose = pose1,
            radius=0.0235,
            half_length=0.05,
            color = (1,0.5,0),
            name = "left_cylinder"
        )



        tag = np.random.randint(0,2)
        pose = rand_pose(
            xlim=[0.15,0.25],
            ylim=[0.02,0.22],
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
            name = "right_cuboid"
        )
        
        # tag = np.random.randint(0,2)
        tag = 0
        pose1 = rand_pose(
            xlim=[0.15,0.25],
            ylim=[0.,0.25],
            zlim=zlim2[tag],
            qpos=qpose2[tag],
            rotate_rand=True,
            rotate_lim=rotate_lim2[tag]
        )

        while np.sum(pow(pose1.p[:2] - pose.p[:2], 2)) < 0.005:
            pose1 = rand_pose(
                xlim=[0.15,0.25],
                ylim=[0.,0.25],
                zlim=zlim2[tag],
                qpos=qpose2[tag],
                rotate_rand=True,
                rotate_lim=rotate_lim2[tag]
            )

        self.right_cylinder = create_cylinder(
            self.scene,
            pose = pose1,
            radius=0.0235,
            half_length=0.05,
            color = (1,0.5,0),
            name = "right_cylinder"
        )
        self.left_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.left_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.right_cuboid.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.right_cylinder.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001



    def play_once(self):
        while 1:
            self.close_right_gripper()

    def check_success(self):
        return 1
