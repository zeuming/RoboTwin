PICK_APPLE = {
    "task_description": "Pick the apple on the table top down, and lift it up for 5 cm, using left arm.",
    "current_code": '''
            from .base_task import Base_task
            from .utils import *
            import sapien

            class pick_apple(Base_task):

                def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
                    super()._init(**kwags)
                    self.create_table_and_wall() 
                    self.load_robot()
                    self.setup_planner()
                    self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
                    self.pre_move()
                    self.load_actors()

                def pre_move(self):
                    pass

                def load_actors(self): # load apple
                    self.apple, self.apple_data = create_obj(
                        self.scene,
                        pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
                        modelname="035_apple"
                    )
                    self.apple_data.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality

                def play_once(self):
                    pass
                    
                def check_success(self):
                    pass
            '''

}

HAMMER_BEAT = {
    "task_description": "Pick up the hammet and use it to beat the block on the table.The hammer is placed at a fixed position on the table, \
                        but the block is generated randomly on the table, if block's x coordinate (dim 0) is greater than 0, use right arm the grasp the hammer, \
                        else use the left arm. You only need to output the content of the play_once() function in the framework.",
    "current_code": '''
                class block_hammer_beat(Base_task):

                    def setup_demo(self,**kwags):
                        # load scence and robote
                        ...

                    def load_actors(self):
                        # load self.hammer, self.hammer_data and self.block
                        ...

                    def play_once(self):
                        # The function that needs to be generated.
                        pass
                        

                    def check_success(self):
                        # check the task success
                        ...
                '''
}   

