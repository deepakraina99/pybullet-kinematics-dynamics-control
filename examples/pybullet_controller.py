# ############################################
# A Robot controller for kinematics, dynamics
# and control based on pyBullet framework
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

# Input:
# 1. robot_type: specify urdf file initials eg. if urdf file name is 'ur5.urdf', specify 'ur5'
# 2. controllable_joints: joint indices of controllable joints. If not specified, by default all joints indices except first joint (first joint is fixed joint between robot stand and base) 
# 3. end-eff_index: specify the joint indices for end-effector link. If not specified, by default the last controllable_joints is considered as end-effector joint
# 4. time_Step: time step for simulation

import pybullet as p
import pybullet_data
import numpy as np
import time

class RobotController:
    def __init__(self, robot_type = 'ur5', controllable_joints = None, end_eff_index = None, time_step = 1e-3):
        self.robot_type = robot_type
        self.robot_id = None
        self.num_joints = None
        self.controllable_joints = controllable_joints
        self.end_eff_index = end_eff_index
        self.time_step = time_step
    # function to initiate pybullet and engine and create world
    def createWorld(self, GUI=True, view_world=False):
        # load pybullet physics engine
        if GUI:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10)
        p.setRealTimeSimulation(True)
        p.loadURDF("plane.urdf")

        #loading robot into the environment
        urdf_file = 'urdf/' + self.robot_type + '.urdf'
        self.robot_id = p.loadURDF(urdf_file, useFixedBase=True)

        self.num_joints = p.getNumJoints(self.robot_id) # Joints
        print('#Joints:',self.num_joints)
        if self.controllable_joints is None:
            self.controllable_joints = list(range(1, self.num_joints-1))
        print('#Controllable Joints:', self.controllable_joints)
        if self.end_eff_index is None:
            self.end_eff_index = self.controllable_joints[-1]
        print('#End-effector:', self.end_eff_index)

        if (view_world):
            while True:
                p.stepSimulation()
                time.sleep(self.time_step)

    # function to joint position, velocity and torque feedback
    def getJointStates(self):
        joint_states = p.getJointStates(self.robot_id, self.controllable_joints)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    # function for setting joint positions of robot
    def setJointPosition(self, position, kp=1.0, kv=1.0):
        print('Joint position controller')
        zero_vec = [0.0] * len(self.controllable_joints)
        p.setJointMotorControlArray(self.robot_id,
                                    self.controllable_joints,
                                    p.POSITION_CONTROL,
                                    targetPositions=position,
                                    targetVelocities=zero_vec,
                                    positionGains=[kp] * len(self.controllable_joints),
                                    velocityGains=[kv] * len(self.controllable_joints))
        for _ in range(100): # to settle the robot to its position
            p.stepSimulation()        

    # function to solve forward kinematics
    def solveForwardPositonKinematics(self, joint_pos):
        print('Forward position kinematics')

        # get end-effector link state
        eeState = p.getLinkState(self.robot_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        print('End-effector pose:', eePose)
        return eePose

    # function to solve inverse kinematics
    def solveInversePositionKinematics(self, end_eff_pose):
        print('Inverse position kinematics')
        joint_angles =  p.calculateInverseKinematics(self.robot_id,
                                                    self.end_eff_index,
                                                    targetPosition=end_eff_pose[0:3],
                                                    targetOrientation=p.getQuaternionFromEuler(end_eff_pose[3:6]))
        print('Joint angles:', joint_angles)
        return joint_angles

    # function to get jacobian 
    def getJacobian(self, joint_pos):
        eeState = p.getLinkState(self.robot_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        zero_vec = [0.0] * len(joint_pos)
        jac_t, jac_r = p.calculateJacobian(self.robot_id, self.end_eff_index, com_trn, list(joint_pos), zero_vec, zero_vec)
        J_t = np.asarray(jac_t)
        J_r = np.asarray(jac_r)
        J = np.concatenate((J_t, J_r), axis=0)
        print('Jacobian:', J)
        return J

    # function to solve forward velocity kinematics
    def solveForwardVelocityKinematics(self, joint_pos, joint_vel):
        print('Forward velocity kinematics')
        J  = self.getJacobian(joint_pos)
        eeVelocity = J @ joint_vel
        print('End-effector velocity:', eeVelocity)
        return eeVelocity

    #function to solve inverse velocity kinematics
    def solveInverseVelocityKinematics(self, end_eff_velocity):
        print('Inverse velocity kinematics')
        joint_pos, _ , _ = self.getJointStates()
        J  = self.getJacobian(joint_pos)
        if len(self.controllable_joints) > 1:
            joint_vel = np.linalg.pinv(J) @ end_eff_velocity
        else:
            joint_vel = J.T @ end_eff_velocity
        print('Joint velcoity:', joint_vel)
        return joint_vel

    #function to do joint velcoity control
    def JointVelocityControl(self, joint_velocities, sim_time=2, max_force=200):
        print('Joint velocity controller')
        t=0
        while t<sim_time:
            p.setJointMotorControlArray(self.robot_id,
                                        self.controllable_joints,
                                        p.VELOCITY_CONTROL,
                                        targetVelocities=joint_velocities,
                                        forces = [max_force] * (len(self.controllable_joints)))
            p.stepSimulation()
            time.sleep(self.time_step)
            t += self.time_step

    #function to do joint velcoity control
    def endEffectorVelocityControl(self, end_eff_vel, sim_time=2, max_forc=200):
        print('End-effector velocity controller')
        t=0
        while t<sim_time:
            joint_velocities = self.solveInverseVelocityKinematics(end_eff_vel)
            self.JointVelocityControl(joint_velocities)
            p.stepSimulation()
            time.sleep(self.time_step)
            t += self.time_step

    # Function to define GUI sliders (name of the parameter,range,initial value)
    def TaskSpaceGUIcontrol(self, goal, max_limit = 3.14, min_limit = -3.14):
        xId = p.addUserDebugParameter("x", min_limit, max_limit, goal[0]) #x
        yId = p.addUserDebugParameter("y", min_limit, max_limit, goal[1]) #y
        zId = p.addUserDebugParameter("z", min_limit, max_limit, goal[2]) #z
        rollId = p.addUserDebugParameter("roll", min_limit, max_limit, goal[3]) #roll
        pitchId = p.addUserDebugParameter("pitch", min_limit, max_limit, goal[4]) #pitch
        yawId = p.addUserDebugParameter("yaw", min_limit, max_limit, goal[5]) # yaw
        return [xId, yId, zId, rollId, pitchId, yawId]

    def ForceGUIcontrol(self, forces, max_limit = 1.0, min_limit = -1.0):
        fxId = p.addUserDebugParameter("fx", min_limit, max_limit, forces[0]) #force along x
        fyId = p.addUserDebugParameter("fy", min_limit, max_limit, forces[1]) #force along y
        fzId = p.addUserDebugParameter("fz", min_limit, max_limit, forces[2]) #force along z
        mxId = p.addUserDebugParameter("mx", min_limit, max_limit, forces[3]) #moment along x
        myId = p.addUserDebugParameter("my", min_limit, max_limit, forces[4]) #moment along y
        mzId = p.addUserDebugParameter("mz", min_limit, max_limit, forces[5]) #moment along z
        return [fxId, fyId, fzId, mxId, myId, mzId]

    # function to read the value of task parameter
    def readGUIparams(self, ids):
        val1 = p.readUserDebugParameter(ids[0])
        val2 = p.readUserDebugParameter(ids[1])
        val3 = p.readUserDebugParameter(ids[2])
        val4 = p.readUserDebugParameter(ids[3])
        val5 = p.readUserDebugParameter(ids[4])
        val6 = p.readUserDebugParameter(ids[5])
        return np.array([val1, val2, val3, val4, val5, val6])

    # function to get desired joint trajectory
    def getTrajectory(self, thi, thf, tf, dt):
        desired_position, desired_velocity, desired_acceleration = [], [], []
        t = 0
        while t <= tf:
            th=thi+((thf-thi)/tf)*(t-(tf/(2*np.pi))*np.sin((2*np.pi/tf)*t))
            dth=((thf-thi)/tf)*(1-np.cos((2*np.pi/tf)*t))
            ddth=(2*np.pi*(thf-thi)/(tf*tf))*np.sin((2*np.pi/tf)*t)
            desired_position.append(th)
            desired_velocity.append(dth)
            desired_acceleration.append(ddth)
            t += dt
        desired_position = np.array(desired_position)
        desired_velocity = np.array(desired_velocity)
        desired_acceleration = np.array(desired_acceleration)
        return desired_position, desired_velocity, desired_acceleration 
    
    #function to calculate dynamic matrics: inertia, coriolis, gravity
    def calculateDynamicMatrices(self):
        joint_pos, joint_vel, _ = self.getJointStates()
        n_dof = len(self.controllable_joints)
        InertiaMatrix= np.asarray(p.calculateMassMatrix(self.robot_id, joint_pos))
        GravityMatrix = np.asarray(p.calculateInverseDynamics(self.robot_id, joint_pos, [0.0] * n_dof, [0.0] * n_dof))
        CoriolisMatrix = np.asarray(p.calculateInverseDynamics(self.robot_id, joint_pos, joint_vel, [0.0] * n_dof)) - GravityMatrix
        return InertiaMatrix, GravityMatrix, CoriolisMatrix

    # Function to simulate free fall under gravity
    def doFreeFall(self):
        p.setRealTimeSimulation(False)
        # Enable torque control
        p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                    p.VELOCITY_CONTROL, 
                                    forces=np.zeros(len(self.controllable_joints)))


        tau = [0.0] * len(self.controllable_joints) # for free fall under gravity
        while True:
            p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau)
            p.stepSimulation()
            time.sleep(self.time_step)
        p.disconnect()

    # Function to do inverse dynamics simulation
    def doInverseDynamics(self, th_initial, th_final, final_time=2):
        p.setRealTimeSimulation(False)
        # get the desired trajectory
        q_d, dq_d, ddq_d = self.getTrajectory(th_initial, th_final, tf=final_time, dt=self.time_step)
        traj_points = q_d.shape[0]
        print('#Trajectory points:', traj_points)

        # forward dynamics simulation loop
        # for turning off link and joint damping
        for link_idx in range(self.num_joints+1):
            p.changeDynamics(self.robot_id, link_idx, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
            p.changeDynamics(self.robot_id, link_idx, maxJointVelocity=200)

        # Enable torque control
        p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                    p.VELOCITY_CONTROL,
                                    forces=np.zeros(len(self.controllable_joints)))

        kd = 0.7 # from URDF file
        n = 0
        while n < traj_points:
            tau = p.calculateInverseDynamics(self.robot_id, list(q_d[n]), list(dq_d[n]), list(ddq_d[n]))
            # tau += kd * dq_d[n] #if joint damping is turned off, this torque will not be required
            # print(tau)
            
            # torque control  
            p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau)
            theta, _, _ = self.getJointStates()
            print('n:{}::th:{}'.format(n,theta))
            
            p.stepSimulation()
            time.sleep(self.time_step)
            n += 1
        print('Desired joint angles:', th_final)
        p.disconnect()

    # Function to do computed torque control
    def computedTorqueControl(self, th_initial, th_final, final_time=2, controller_gain=400):
        p.setRealTimeSimulation(False)
        # get the desired trajectory
        q_d, dq_d, ddq_d = self.getTrajectory(th_initial, th_final, tf=final_time, dt=self.time_step)
        traj_points = q_d.shape[0]
        print('#Trajectory points:', traj_points)

        # forward dynamics simulation loop
        # for turning off link and joint damping
        for link_idx in range(self.num_joints+1):
            p.changeDynamics(self.robot_id, link_idx, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
            p.changeDynamics(self.robot_id, link_idx, maxJointVelocity=200)

        # Enable torque control
        p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                    p.VELOCITY_CONTROL,
                                    forces=np.zeros(len(self.controllable_joints)))        

        Kp = controller_gain
        Kd = 2 * np.sqrt(Kp)
        n=0
        while n < q_d.shape[0]:

            # get current joint states
            q, dq, _ = self.getJointStates()
            # PD control
            q_e = q_d[n] - np.asarray(q)
            dq_e = dq_d[n] - np.asarray(dq)
            aq = ddq_d[n] + Kp * q_e + Kd * dq_e

            tau = p.calculateInverseDynamics(self.robot_id, list(q), list(dq), list(aq))
            # tau += kd * dq_d[n] # if joint damping is turned off, this torque will not be required
            # print(tau)
            
            # torque control  
            p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau)
            
            print('n:{}::th:{}'.format(n,q))

            p.stepSimulation()
            time.sleep(self.time_step)
            n += 1
        print('Desired joint angles:', th_final)
        p.disconnect()

    # Function to do impedence control in task space
    def impedenceController(self, th_initial, desired_pose, controller_gain=100):
        p.setRealTimeSimulation(False)
        # forward dynamics simulation loop
        # for turning off link and joint damping
        for link_idx in range(self.num_joints+1):
            p.changeDynamics(self.robot_id, link_idx, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
            p.changeDynamics(self.robot_id, link_idx, maxJointVelocity=200)

        # Enable torque control
        p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                    p.VELOCITY_CONTROL, 
                                    forces=np.zeros(len(self.controllable_joints)))

        kd = 0.7 # from URDF file
        Kp = controller_gain
        Kd = 2 * np.sqrt(Kp)
        Md = 0.01*np.eye(6)

        # Target position and velcoity
        # xd = np.array([0.4499998573193256, 0.1, 1.95035834701983, 0.0, 1.5707963267948966, 0.0]) #1-link
        # xd = np.array([1.3499995719791142, 0.2, 2.9510750145148816, 0.0, 1.5707963267948966, 0.0]) #2-link
        # xd = np.array([2.199999302511422, 0.3, 2.9517518416742643, 0.0, 1.5707963267948966, 0.0]) #3-link
        # xd = np.array([1.8512362079506117, 0.30000000000000004, 4.138665008474901, -0.0, 1.0000000496605894, -0.0]) #3-link
        # xd = np.array([0.10972055742719365, -0.716441307051838, 1.44670878280948, -1.5700006464761673, 0.0007970376813496536, -1.570796326772595]) #ur5-link
        # xd = np.array([0.6811421738723965, -0.24773390188802563, 1.44670878280948, -1.5700006464761678, 0.0007970376813495148, -0.5007963267725951]) #ur5-link
        # xd = np.array([-0.10857937593446423, 0.7166151451748437, 1.4467087828094798, -1.5700006464761673, 0.0007970376813502642, 1.5692036732274044]) #ur5-link
        xd = desired_pose
        dxd = np.zeros(len(self.controllable_joints))

        # define GUI sliders
        xdGUIids = self.TaskSpaceGUIcontrol(goal=xd)
        ForceInitial = np.zeros(len(self.controllable_joints))
        ForceGUIids = self.ForceGUIcontrol(forces=ForceInitial, max_limit=10, min_limit=-10) 

        while True:
            # read GUI values
            xd = self.readGUIparams(xdGUIids) # task space goal
            F_ext = self.readGUIparams(ForceGUIids) # applied external forces

            # get current joint states
            q, dq, _ = self.getJointStates()

            # Error in task space
            x = self.solveForwardPositonKinematics(q)
            x_e = xd - x
            dx = self.solveForwardVelocityKinematics(q, dq)
            dx_e = dxd - dx

            # Task space dynamics
            # Jacobian    
            J = self.getJacobian(q)
            J_inv = np.linalg.pinv(J)
            # Inertia matrix in the joint space
            Mq, G, _ = self.calculateDynamicMatrices()
            # Inertia matrix in the task space
            Mx = np.dot(np.dot(np.transpose(J_inv), Mq), J_inv)
            # Force in task space
            Fx = np.dot(np.dot(np.linalg.inv(Md), Mx),(np.dot(Kp, x_e) + np.dot(Kd, dx_e)))
            # External Force applied
            F_w_ext = np.dot((np.dot(np.linalg.inv(Md), Mx) - np.eye(6)), F_ext)
            Fx += F_w_ext
            # Force in joint space
            Fq = np.dot(np.transpose(J),Fx) 

            # Controlled Torque
            tau = G + Fq
            # tau += kd * np.asarray(dq) # if joint damping is turned off, this torque will not be required
            # print('tau:', tau)
            
            # Activate torque control  
            p.setJointMotorControlArray(self.robot_id, self.controllable_joints,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau)

            p.stepSimulation()
            time.sleep(self.time_step)
        p.disconnect()
