from pxr import UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools, UsdGeom
import omni
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
import time


timeline = omni.timeline.get_timeline_interface()
dc = _dynamic_control.acquire_dynamic_control_interface()
current_time = timeline.get_current_time()
forklift_art = dc.get_articulation("/World/forklift_b_rigged_cm")

body_ptr = dc.find_articulation_body(forklift_art, "body")
dof_ptr1 = dc.find_articulation_dof(forklift_art, "back_wheel_drive")
body_pose = dc.get_rigid_body_pose(body_ptr)

if current_time > 2.0:
	if body_pose.p[0] < -4.5 and body_pose.p[0] < -2.0:
		dc.set_dof_velocity_target(dof_ptr1, 1.0)
	elif body_pose.p[0] > -2.0 and body_pose.p[0] > -4.5:
		dc.set_dof_velocity_target(dof_ptr1, -1.0)

