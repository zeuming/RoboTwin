
from .base_task import Base_task
from .utils import *
import sapien

class hammer_beat(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.step_lim = 150

    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.hammer = create_obj(
            self.scene,
            pose=sapien.Pose([0.25, -0.05, 0.78],[0, -0.05, 1, 0.23]),
            modelname="081_hammer_2",
            scale=(0.063,0.079,0.079)
        )
        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.close_right_gripper()
        self.open_right_gripper()
        pose0 = list(self.hammer.get_pose().p+[0,-0.08,0.18])+[-0.591,0.425,-0.320,-0.606]
        self.right_move_to_pose_with_screw(pose0,save_freq=None)
        pose0[2] -=0.05
        self.right_move_to_pose_with_screw(pose0,save_freq=None)
        self.close_right_gripper(save_freq=None)
        init_pose = [0.301, -0.216, 0.988, -0.683, 0.183, -0.183, -0.683]
        self.right_move_to_pose_with_screw(init_pose,save_freq=None)
        pose1 = [0.301, -0.216, 1.018, -0.683, 0.183, -0.183, -0.683]
        self.right_move_to_pose_with_screw(pose1,save_freq=None)
        self.render_freq = render_freq

    def load_actors(self):
        coaster_pose = rand_pose(
            xlim=[0,0.3],
            ylim=[-0.05,0.2],
            zlim=[0.76],
            qpos=[-0.552, -0.551, -0.442, -0.444],
            rotate_rand=True,
            rotate_lim=[0,1,0],
        )
        self.coaster = create_box(
            scene = self.scene,
            pose = coaster_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            name="box"
        )
        self.coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001

    def play_once(self):
        pose1 = list(self.coaster.get_pose().p+[0.056,-0.095,0.27])+[-0.378,0.559,-0.316,-0.667]
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        pose1[2]-=0.06
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        for  _ in range(2):
            self._take_picture()
        

    def is_success(self):
        hammer_pose = self.hammer.get_pose().p
        coaster_pose = self.coaster.get_pose().p
        eps = 0.02
        return abs(hammer_pose[0]-coaster_pose[0]-0.03)<eps and abs(hammer_pose[1] + 0.06 - coaster_pose[1])<eps and hammer_pose[2]>0.8 and hammer_pose[2] < 0.83