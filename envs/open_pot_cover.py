
from .base_task import Base_task
from .utils import *
import math
import sapien

class open_pot_cover(Base_task):
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
        # super().setup_scene()
        # self.target = create_visual_box(
        #     scene = self.scene,
        #     pose = sapien.Pose([0,-0.08,0.74],[1,0,0,0]),
        #     half_size=(0.13,0.05,0.0005),
        #     color=(0,0,1),
        #     name="box"
        # )

        # shoes_pose = rand_pose(
        #     xlim=[-0.25,0.25],
        #     ylim=[-0.1,0.05],
        #     zlim=[0.8],
        #     ylim_prop=True,
        #     rotate_rand=True,
        #     rotate_lim=[0,3.14,0],
        #     qpos=[0.707,0.707,0,0]
        # )

        # while np.sum(pow(shoes_pose.get_p()[:2] - np.zeros(2),2)) < 0.0225:
        #     shoes_pose = rand_pose(
        #         xlim=[-0.25,0.25],
        #         ylim=[-0.1,0.05],
        #         zlim=[0.8],
        #         ylim_prop=True,
        #         rotate_rand=True,
        #         rotate_lim=[0,3.14,0],
        #         qpos=[0.707,0.707,0,0]
        #     )
        

        # self.shoe, self.shoe_data = create_glb(
        #     self.scene,
        #     pose=shoes_pose,
        #     modelname="041_shoes",
        #     convex=True,
        #     model_id = np.random.choice(self.id_list),
        #     model_z_val = True
        # )

        # self.shoe.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        self.pot,_ = create_urdf_obj(
            self.scene,
            pose = sapien.Pose([0,0,0.8],[1,0,0,1]),
            modelname="100015",
            scale=0.1,
            fix_root_link=False,
        )
        self.pot_active_joints = self.pot.get_active_joints()
        for joint in self.pot_active_joints:
            joint.set_drive_property(stiffness=20, damping=5, force_limit=1000, mode="force")
            print(joint.get_name())
        self.pot_all_joints = self.pot.get_joints()

    def play_once(self):
        while 1:
            self.close_right_gripper()

    def check_success(self):
        return 1
