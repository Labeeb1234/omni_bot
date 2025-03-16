import numpy as np
from omni_bot.utils.utils import WrapToPi, get_transform

class OmniBot:
    def __init__(self, a, L, W, t, m, Icz, com_vec=[0.0, 0.0], inital_pos=(0,0,0)):
        self.a, self.l, self.w, self.t = a, L, W, t
        self.m = m
        self.Icz = Icz 
        # centre of mass pose/centre of gravity pose
        self.pose_com = np.array(com_vec).reshape((2,1)) # assuming 2D-motion dynamics

        # wheel config matrix (forward)
        self.W = np.array([
            [0, -self.a/2, 0, self.a/2],
            [self.a/2, 0, -self.a/2, 0],
            [(self.a/2)*((self.l+self.t)/((self.l+self.t)**2+(self.w+self.t)**2)), 
             (self.a/2)*((self.w+self.t)/((self.l+self.t)**2+(self.w+self.t)**2)),
             (self.a/2)*((self.l+self.t)/((self.l+self.t)**2+(self.w+self.t)**2)),
             (self.a/2)*((self.w+self.t)/((self.l+self.t)**2+(self.w+self.t)**2))]
        ]).reshape(3,4)
        
        # based on COM
        self.wheel_positions = np.array([
            [self.l + (self.t/2), 0], # w1
            [0, self.w + (self.t/2)], # w2
            [-(self.l + (self.t/2)), 0], # w3,
            [0, -(self.w + (self.t/2))] # w4
        ], dtype=float)

        self.pose = np.array([0.0, 0.0, 0.0]).reshape((3,1))

    def get_bot_outline(self):
        # note: theta=phi (body and global frame yaws same since motion in a plane)
        # initial body outline based on the body-frame
        body = np.array([
            [-self.l, -self.w],
            [self.l, -self.w],
            [self.l, self.w],
            [-self.l, self.w]
        ]).T

        trans, rotz = get_transform(state=self.pose)
        # print(f"tvec: {trans}")
        # print(f"rmat: {rotz}")

        body_glob = rotz @ body
        body_glob = body_glob.T
        body_glob[:, 0] += trans[0, 0]
        body_glob[:, 1] += trans[1,0]

        return body_glob # global

    def get_wheel_positions(self):
        wheels = self.wheel_positions

        # Construct the rotation matrix
        # Rz = np.array([
        #     [np.cos(self.pose[2,0]), -np.sin(self.pose[2,0])],
        #     [np.sin(self.pose[2,0]),  np.cos(self.pose[2,0])]
        # ])
        trans, rotz = get_transform(state=self.pose)

        wheels_glob = rotz @ wheels.T
        wheels_glob = wheels_glob.T
        wheels_glob[:, 0] += trans[0,0]
        wheels_glob[:, 1] += trans[1,0]

        return wheels_glob
    
    # ---------------- Kinematics --------------- 
    def inverse_kinematics(self, body_vel): # body_vel --> zeta
        U = np.array(body_vel).reshape((3,1)) # [u, v, r]

        # wheel_config_matrix
        W_star = np.array([
            [0, 1/self.a, (self.l+self.t)/self.a],
            [-1/self.a, 0, (self.w+self.t)/self.a],
            [0, -1/self.a, (self.l+self.t)/self.a],
            [1/self.a, 0, (self.w+self.t)/self.a]
        ], dtype=float).reshape((4,3)) 

        # calculated wheel velocities
        omega = W_star @ U

        return omega

    def forward_kinematics(self, omega):
        J = np.array([
            [np.cos(self.pose[2,0]), -np.sin(self.pose[2,0]), 0],
            [np.sin(self.pose[2,0]),  np.cos(self.pose[2,0]), 0],
            [0,                 0,                 1]
        ])

        vel_global = J @ (self.W @ omega) # ---> zeta vector (body frame velocity vector)

        return vel_global
    

    # ------------------------------------------
    def update_odom(self, vel_global, dt):
        self.pose[0,0] += vel_global[0, 0] * dt
        self.pose[1,0] += vel_global[1, 0] * dt
        self.pose[2,0] += vel_global[2, 0] * dt
        self.pose[2,0] = WrapToPi(self.pose[2,0])

    def dynamic_odom_update(self, vel_global, body_accel, dt):
        # note: vel_global --> eta_dot
        J = np.array([
            [np.cos(self.pose[2,0]), -np.sin(self.pose[2,0]), 0],
            [np.sin(self.pose[2,0]),  np.cos(self.pose[2,0]), 0],
            [0,                 0,                 1]
        ])
        accel_global = (J @ body_accel).reshape((3,1))
        self.pose[0,0] += dt*(vel_global[0, 0] + 0.5*accel_global[0, 0]*dt) 
        self.pose[1,0] += dt*(vel_global[1, 0] + 0.5*accel_global[1, 0]*dt) 
        self.pose[2,0] += dt*(vel_global[2, 0] + 0.5*accel_global[2, 0]*dt) 
        self.pose[2,0] = WrapToPi(self.pose[2,0])


    # ---------------- Dynamics ---------------
    def forward_dynamics(self, tau, n_v):
        tau = np.array(tau).reshape((3,1))
        # zeta --> body_vel [u, v, r]
        # tau ---> input force/ the control input or the net external forces acting on the system [Fxb, Fyb, Mzb]
        # D ---> inertia matrix
        D = np.array([
            [self.m, 0, -self.m*self.pose_com[1,0]],
            [0, self.m, self.m*self.pose_com[0,0]],
            [-self.m*self.pose_com[1,0], self.m*self.pose_com[0,0], self.Icz+self.m*(self.pose_com[0,0]**2+self.pose_com[1,0]**2)]
        ]).reshape((3,3))
        o_vec = np.array(n_v).reshape(3,1)
        '''
            n_v(zeta) = [
                [-mr*(v+xbc*r), mr(u-ybc*r), mr(xbc*u+ybc*v)]
            ] 
        '''

        # tau =  D @ zeta_dot + o_vec # o_vec --> n(zeta)
        zeta_dot = np.linalg.inv(D) @ (tau - o_vec)

        return zeta_dot

    def inverse_dynamics(self, body_vel):

        pass

    def wheel_config_dynamics(self):
        pass



