import carb # type:ignore
import omni.kit.window.property # type: ignore
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS #type: ignore
from isaacsim.replicator.behavior.utils.behavior_utils import ( # type: ignore
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.kit.scripting import BehaviorScript #type:ignore
from isaacsim.core.prims import Articulation #type:ignore
from pxr import Sdf, Usd #type:ignore
import numpy as np
'''

Custom Behaviour Script/node to control the routine behaviour a forklift articulation

'''

class ForkLiftMotion(BehaviorScript):
    BEHAVIOUR_NS = "ForkLift_Motion"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 1,
            "doc": "Interval Rate for Forklift logic update function"
        },
        {
            "attr_name": "forklift_velocity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.1,
            "doc": "For setting the back joint drive velocity/not articulation body velocity; skipping on kinematic logic of control for now"
        },
        {
            "attr_name": "start_dynamic",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": False,
            "doc": "whether to make the forklift a dynamic prim or not"
        },
    ]

    def on_init(self):
        carb.log_info(f"on_init()->{self.prim}")
        self.interval = 1
        self.forklift_joint_vel = 0.1
        self.update_counter = 0
        self._articulation_prim_initialized = False
        self._start_dynamic = False

        # exposing variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
        # refreshing isaacim property windows
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info(f"on_destroy()->{self.prim_path}")
        self._reset()
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
            omni.kit.window.property.get_window().request_rebuild()

    def on_play(self):
        carb.log_info(f"on_play()->{self.prim_path}")
        self._setup()
        
    def on_pause(self):
        carb.log_info(f"on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"on_stop()->{self.prim_path}")
        self._reset()

    def on_update(self, current_time: float, delta_time: float):
        if delta_time <= 0:
            return
        
        self.update_counter += 1
        if self.update_counter >= self.interval:
            # carb.log_info(f"on_update->({current_time}, {delta_time})->{self.prim_path}")
            if self._start_dynamic:
                # j1_idx = self.forklift_art.get_joint_index('back_wheel_swivel')
                j2_idx = self.forklift_art.get_joint_index('back_wheel_drive')
                current_pos, current_rotq = self.forklift_art.get_world_poses()
                # print(f"back drive joint pos: {np.squeeze(self.forklift_art.get_joint_positions(), axis=0)[j2_idx]}")
                # print(f"back drive vel: {np.squeeze(self.forklift_art.get_joint_velocities(), axis=0)[j2_idx]}\n")
                if (np.abs(current_pos[0, 0]+6.8) < 0.01) or (np.abs(current_pos[0, 0]+1.204) < 0.01):
                    print(f"[INFO]: Forklift Reached pos: {current_pos[0,0]}")

                # using PD-controller based joint targets function/sub-routine
                if current_pos[0, 0] <= -6.8:
                    self.forklift_art.set_joint_velocity_targets(
                        velocities=np.array([self.forklift_joint_vel]),
                        joint_indices=np.array([j2_idx])
                    )
                elif current_pos[0, 0] >= -1.204:
                    self.forklift_art.set_joint_velocity_targets(
                        velocities=np.array([-self.forklift_joint_vel]),
                        joint_indices=np.array([j2_idx])
                    )

                # start pose: pos=(array([[-6.8763127 ,  2.3894968 ,  0.00093228]], dtype=float32), quat(w,x,y,z)=array([[-0.00000145,  0.00008491, -0.00000448,  1.0000001 ]]
                # ref pose-1: pos(xyz)=(array([[-1.2039996 , 2.3981142 , 0.00093144]]), quat(w,x,y,z)=array([[-0.00085215,  0.00008581, -0.00000332,  0.99999976]]

            self.update_counter = 0

    # helper class methods/attributes
    def _apply_behaviour(self):
        if self._start_dynamic:
            j2_idx = self.forklift_art.get_joint_index('back_wheel_drive')
            current_pos, current_rotq = self.forklift_art.get_world_poses()
            # goal pos checker
            if (np.abs(current_pos[0, 0]+6.8) < 0.01) or (np.abs(current_pos[0, 0]-1.204) < 0.01):
                print(f"[INFO]: Forklift Reached pos: {current_pos[0,0]}")

            # using PD-controller based joint targets function/sub-routine
            if current_pos[0, 0] <= -6.8:
                self.forklift_art.set_joint_velocity_targets(
                    velocities=np.array([self.forklift_joint_vel]),
                    joint_indices=np.array([j2_idx])
                )
            elif current_pos[0, 0] >= -1.204:
                self.forklift_art.set_joint_velocity_targets(
                    velocities=np.array([-self.forklift_joint_vel]),
                    joint_indices=np.array([j2_idx])
                )


    def _setup(self):
        self.interval = self._get_exposed_variable(self.VARIABLES_TO_EXPOSE[0]['attr_name'])
        self.forklift_joint_vel = self._get_exposed_variable(self.VARIABLES_TO_EXPOSE[1]['attr_name'])
        self._start_dynamic = self._get_exposed_variable(self.VARIABLES_TO_EXPOSE[2]['attr_name'])
        role = "Dynamic" if self._start_dynamic else "Static"
        print(f"[INFO]: Forklift role: {role}")
        print(f"[INFO]: Setting Logic Update Interval as: {self.interval}")
        print(f"[INFO]: Setting Forklift Back Joint Drive Velocity as: {self.forklift_joint_vel:.2f}")
        
        # setting articulation wrapper around the forklift prim
        if not self._articulation_prim_initialized:
            prim_path = self.prim.GetPath().pathString
            self.forklift_art = Articulation(
                prim_paths_expr=prim_path,
                name='forklift_articulation_view'   
            )
            self._articulation_prim_initialized = True
            carb.log_info(f"forklift_b articulation prim initialized at prim path: {prim_path}")
            
        if not self.prim.IsValid():
            carb.log_info("prim is not valid")

    def _reset(self):
        self._start_dynamic = False
        self._articulation_prim_initialized = False
        self.forklift_art = None
        self.update_counter = 0
        self.interval = 1

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOUR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)