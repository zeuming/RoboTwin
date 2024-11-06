from .base_task import Base_task
from .utils import *
import sapien


class apple_cabinet_storage(Base_task):
    def setup_demo(self,**kwags):
        super()._init(**kwags,table_static=False)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
        self.step_lim = 600

    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq
    
    def load_actors(self):
        self.cabinet, self.cabinet_data = rand_create_urdf_obj(
            self.scene,
            modelname="036_cabine",
            xlim=[0,0.01],
            ylim=[0.13,0.15],
            zlim=[0.96],
            rotate_rand=False,
            qpos=[1,0,0,1],
            # qpos=[0,0.707,0.707,0],
            scale=0.27
        )

        self.cabinet_active_joints = self.cabinet.get_active_joints()
        for joint in self.cabinet_active_joints:
            joint.set_drive_property(stiffness=20, damping=5, force_limit=1000, mode="force")
        self.cabinet_all_joints = self.cabinet.get_joints()

        self.apple,self.apple_data = rand_create_obj(
            self.scene,
            xlim=[0.2,0.32],
            ylim=[-0.17,-0.05],
            zlim=[0.78],
            modelname="035_apple",
            rotate_rand=False,
            convex=True
        )
        self.apple.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.actor_data_dic = {"cabinet": self.cabinet_data, "apple": self.apple_data}
        self.actor_name_dic = {"cabinet": self.cabinet, "apple": self.apple} 
        
    def play_once(self):
        pass
        
    def check_success(self):
        cabinet_pos = self.cabinet.get_pose().p
        eps = 0.03
        apple_pose = self.apple.get_pose().p
        left_endpose = self.get_left_endpose_pose()
        target_pose = (cabinet_pos - np.array([0, 0.268, 0.09])).tolist() + [0.5, -0.5, -0.5, 0.5]
        eps1 = 0.02
        return np.abs(apple_pose[2] - 0.797) < 0.015 and \
               np.all(abs(np.array(left_endpose.p.tolist() + left_endpose.q.tolist()) - target_pose) < eps1)