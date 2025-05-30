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


class ForkLiftMotion(BehaviorScript):
    BEHAVIOUR_NS = "ForkLift_Motion"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 10,
            "doc": "Interval Rate for Forklift logic update function"
        },
        {
            "attr_name": "forklift_velocity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.1,
            "doc": "For setting the velocity of forklift articulation"
        },
    ]

    def on_init(self):
        carb.log_info(f"on_init()->{self.prim}")
        self.interval = 10
        self.forklift_vel = 0.1
        self.update_counter = 0
        self._articulation_prim_initialized = False


        # exposing variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
        # refreshing isaacim property windows
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
            omni.kit.window.property.get_window().request_rebuild()

    def on_play(self):
        carb.log_info(f"on_play()->{self.prim}")
        self._setup()
        
    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        if delta_time <= 0:
            return
        
        self.update_counter += 1
        if self.update_counter >= self.interval:
            carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")
            self.update_counter = 0
    
    def _setup(self):
        self.interval = self._get_exposed_variable(self.VARIABLES_TO_EXPOSE[0]['attr_name'])
        self.forklift_vel = self._get_exposed_variable(self.VARIABLES_TO_EXPOSE[1]['attr_name'])

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
        self._articulation_prim_initialized = False
        self.forklift_art = None
        self.update_counter = 0
        self.interval = 10

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOUR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)