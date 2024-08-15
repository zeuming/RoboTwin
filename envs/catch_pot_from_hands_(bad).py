
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import sapien


class catch_pot_from_hands(Base_task):
    """
    This is the most basic demo of the motion planning library where the robot tries to
    shuffle three boxes around.
    """
    
    def setup_demo(self,st_ep_num=0,seed = 0,render_fre = 0):
        super().__init__(save_dir="./czx_benchmark_data/",task_name="catch_pot_from_hands",st_ep_num=st_ep_num,seed = seed, render_fre=render_fre)
        self.setup_scene()
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera()

        robot_pose_in_world = [0,-0.65,0,1,0,0,1]
        self.left_planner.set_base_pose(robot_pose_in_world)
        self.right_planner.set_base_pose(robot_pose_in_world)

    def setup_scene(self, **kwargs):
        """
        create apple and plate, add them into actor_list[]
        """
        super().setup_scene()
        self.right_hand = rand_create_obj(
            self.scene,
            xlim=[-0.1,0.05],
            ylim=[0.,0.1],
            zlim=[0.8],
            modelname="091_right_hand",
            rotate_rand=False,
            qpos=[-0.194,-0.021,0.086,0.977],
            scale= (0.105,0.105,0.105),
            convex=True,
            is_static=True
        )

        self.left_hand=create_obj(
            self.scene,
            pose= sapien.Pose([self.right_hand.pose.p[0]+0.173, self.right_hand.pose.p[1]+0.03, self.right_hand.pose.p[2]],[0.701,0.093,-0.093,-0.701]),
            modelname = "092_left_hand",
            scale = (0.12,0.12,0.12),
            convex=True,
            is_static=True
        )

        self.pot=create_obj(
            self.scene,
            pose= sapien.Pose([self.right_hand.pose.p[0]+0.07079663,self.right_hand.pose.p[1]-0.018,self.right_hand.pose.p[2]+0.099],[0.748,0.662,0.011,-0.039]),
            modelname = "093_pot",
            scale = (0.15,0.15,0.15)
        )
        self.pot.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        
    def play_once(self,save_freq=None):

        self.together_open_gripper(save_freq=None)
        left_pose = list(self.pot.get_pose().p+[-0.29,-0.01,0.1])+[-0.706,0.697,-0.054,-0.111]
        right_pose = list(self.pot.get_pose().p+[0.305,-0.025,0.07])+[-0.102,-0.06,-0.693,-0.711]
        left_target_pose = list([-0.24,-0.21,1.06])+[-0.706,0.697,-0.054,-0.111]
        right_target_pose = list([0.255,-0.225,1.03])+[-0.102,-0.06,-0.693,-0.711]
        # pose3 = [0,-]
        self.together_move_to_pose_with_screw(left_pose,right_pose,save_freq=save_freq)
        left_pose[0]+=0.05
        right_pose[0]-=0.05
        self.together_move_to_pose_with_screw(left_pose,right_pose,save_freq=save_freq)
        self.together_close_gripper(pos=-0.005,save_freq=save_freq)
        left_pose[2]+=0.07
        right_pose[2]+=0.07
        self.together_move_to_pose_with_screw(left_pose,right_pose,save_freq=save_freq)
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=save_freq)
        left_target_pose[2]-=0.145
        right_target_pose[2]-=0.15
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=save_freq)
        self.together_open_gripper(save_freq=save_freq)
        left_target_pose[0]-=0.05
        right_target_pose[0]+=0.05
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=save_freq)
        self.together_move_to_pose_with_screw(self.left_original_pose,self.right_original_pose,save_freq=save_freq)
        # self.together_close_gripper(save_freq=None)

    
    def check_success(self):
        target_pose = [-0.01, -0.2, 0.814]
        target_rotate = 0.707
        pot_pose = self.pot.get_pose().p
        pot_rotate = self.pot.get_pose().q
        eps0 = 0.05
        eps1 = 0.005
        # print(pot_pose)
        # print(pot_rotate)
        return abs(target_pose[0]-pot_pose[0])<eps0 and abs(target_pose[1]-pot_pose[1])<eps0 and abs(target_pose[2]-pot_pose[2])<eps1 \
            and abs(pot_rotate[0] - target_rotate)<eps1 and abs(pot_rotate[1] - target_rotate)<eps1
        # bowl_xlim = [self.actor_list[0].get_pose().p[0]-0.05,self.actor_list[0].get_pose().p[0]+0.05]
        # bowl_ylim = [self.actor_list[0].get_pose().p[1]-0.05,self.actor_list[0].get_pose().p[1]+0.05]
        # return self.apple_obj.get_pose().p[0] > bowl_xlim[0] and self.apple_obj.get_pose().p[0] < bowl_xlim[1] and self.apple_obj.get_pose().p[1] > bowl_ylim[0]
