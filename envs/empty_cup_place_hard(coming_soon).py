
from .base_task import Base_task
from .utils import *
import sapien

class empty_cup_place_hard(Base_task):
    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.step_lim = 500
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.open_right_gripper()
        self.render_freq = render_freq

    def load_actors(self):
        self.cup,self.cup_data = rand_create_glb(
            self.scene,
            xlim=[0.15,0.3],
            ylim=[-0.2,0.05],
            zlim=[0.8],
            modelname="022_cup",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
        )
        cup_pose = self.cup.get_pose().p

        coaster_pose = rand_pose(
            xlim=[-0.05,0.1],
            ylim=[-0.2,0.05],
            zlim=[0.76],
            rotate_rand=False,
            rotate_lim=[0,3.14,0],
            qpos=[0.707,0.707,0,0],
        )

        while np.sum(pow(cup_pose[:2] - coaster_pose.p[:2],2)) < 0.01:
            coaster_pose = rand_pose(
                xlim=[-0.05,0.1],
                ylim=[-0.2,0.05],
                zlim=[0.76],
                rotate_rand=False,
                rotate_lim=[0,3.14,0],
                qpos=[0.707,0.707,0,0],
            )
        self.coaster,_ = create_obj(
            self.scene,
            pose = coaster_pose,
            modelname="019_coaster",
            convex=True
        )
        
        self.cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def play_once(self):

        while 1:
            self.close_left_gripper()
        pose0 = list(self.cup.get_pose().p+[0.048,0,0.245])+[-0.557,0.473,-0.473,-0.489]
        self.right_move_to_pose_with_screw(pose0,save_freq=15)

        # print(self.cup.get_pose().p)
        self.close_right_gripper(pos = 0.02,save_freq=15)
        pose0[2] -=0.08
        self.right_move_to_pose_with_screw(pose0,save_freq=15)
        self.close_right_gripper(pos = -0.01,save_freq=15)
        pose0[2] +=0.09
        self.right_move_to_pose_with_screw(pose0,save_freq=15)
        pose1 = list(self.coaster.get_pose().p+[0.035,-0.02,0.3])+[-0.557,0.473,-0.473,-0.489]
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        pose1[2] -=0.082
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        self.open_right_gripper(pos=0.02,save_freq=15)
        pose1[2] +=0.06
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        for _ in range(2):
            self._take_picture()
    
    def check_success(self):
        eps = 0.025
        coaster_pose = self.coaster.get_pose().p
        cup_pose = self.cup.get_pose().p
        # print(cup_pose)
        return abs(cup_pose[0] - coaster_pose[0])<eps  and  abs(cup_pose[1] - coaster_pose[1])<eps and (cup_pose[2] - 0.792) < 0.005
