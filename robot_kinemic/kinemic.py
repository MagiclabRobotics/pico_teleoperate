import pinocchio
import numpy as np
from numpy.linalg import norm, solve

import robot_kinemic.plot_util
from robot_kinemic.conf import *
from robot_kinemic.kinemic_utils import *
# from robot_kinemic.log.log_manage import log_m
from robot_kinemic.config_info import Config

eps = 0.01
IT_MAX = 1000
DT = 5e-1
damp = 1e-12
import copy
PI = 3.1415926

# SensorPosition = "Thumb_side"


class loadUrdf():
    def __init__(self, is_left=True, config=None):
        self.is_left = is_left
        self.config = config
        if config is None:
            self.config = Config()

        if (is_left):
            with pkg_resources.path(ROBOT_KINEMIC_MODEL_PATH, L_ARM_URDF_NAME) as urdf_path:
                self.urdf_file_path = str(urdf_path)
            # self.urdf_file_path = LEFT_ARM_URDF
            print(f"LEFT_ARM_URDF: {urdf_path}")
        else:
            with pkg_resources.path(ROBOT_KINEMIC_MODEL_PATH, R_ARM_URDF_NAME) as urdf_path:
                self.urdf_file_path = str(urdf_path)
            # self.urdf_file_path = RIGHT_ARM_URDF
            print(f"RIGHT_ARM_URDF: {urdf_path}")
        self.dim = 6
        self.num_link = 7
        self.end_idx = 5
        self.pos = np.zeros(3)
        self.eul = np.zeros(3)
        self.tp = np.zeros(self.dim)

        self.elbow_pos = np.zeros(3)
        self.elbow_eul = np.zeros(3)

        # if(self.is_left):
        #     self.qpos_threshold = [[-2.7576,2.7576], [-0.2618, 2.5656], [-2.6354, 2.6354], [-2.2166,0.2443], [ -2.9147, 2.9147], [-0.9948, 0.9948], [-0.4538, 0.4538]]
        # else:
        #     self.qpos_threshold = [[-2.7576,2.7576], [-2.5656, 0.2618], [-2.6354, 2.6354], [-0.2443, 2.2166], [ -2.9147, 2.9147], [-0.9948, 0.9948], [-0.4538, 0.4538]]

        # if(self.is_left):
        #     self.qpos_threshold = [[-2.7576,2.7576], [0, 2.5656], [-2.6354, 2.6354], [-2.2166, 0.2443], [ -2.9147, 2.9147], [0, 0], [0, 0]]
        # else:
        #     self.qpos_threshold = [[-2.7576,2.7576], [-2.5656, 0], [-2.6354, 2.6354], [-0.2443, 2.2166], [ -2.9147, 2.9147], [0, 0], [0, 0]]

        # #肱二头肌限位暂时缩小

        # if(self.is_left):
        #     self.qpos_threshold = [[-1.8,0.5], [0, 2.5656], [-2., 2.], [-2.2166, -0.], [ -2.9147, 2.9147], [0, 0], [0, 0]]
        # else:
        #     self.qpos_threshold = [[-1.8,0.5], [-2.5656, 0], [-2., 2.], [0., 2.2166], [ -2.9147, 2.9147], [0, 0], [0, 0]]
            
        if(self.is_left):
            self.qpos_threshold = self.config.arm_l
        else:
            self.qpos_threshold = self.config.arm_r
            

        self.deta_h = [0 for _ in range(self.num_link)]

        self.vel = [0 for _ in range(self.num_link)]
        self.acc = [0 for _ in range(self.num_link)]

        self.weight = np.eye(self.num_link)
        self.iter = 10
        self.err_threshold = 0.01
        self.init_tp = np.zeros(self.dim)

        self.qpos = np.zeros(self.num_link)
        self.load_urdf()
        self.iteration = self.inv_kinemic()

    def load_urdf(self):
        print(f"self.urdf_file_path: {self.urdf_file_path}")
        self.model = pinocchio.buildModelFromUrdf(self.urdf_file_path)
        self.data = self.model.createData()

        qpos = pinocchio.neutral(self.model)
        if self.config.sensor_position == "Back_of_the_hand":
            #传感器在手背
            if(self.is_left):
                qpos[2] = 0.5* PI
                qpos[3] = -0.5* PI
                qpos[4] = -PI
            else:
                qpos[2] = -0.5* PI
                qpos[3] = 0.5* PI
                qpos[4] = PI
        elif self.config.sensor_position == "Palm_of_the_hand":
            # 传感器在手心测 传感器运动方向 发生改变 update_target_pos_diff 有调整
            if(self.is_left):
                qpos[2] = 0.5* PI
                qpos[3] = -0.5* PI
                qpos[4] = -PI*0.0
            else:
                qpos[2] = -0.5* PI
                qpos[3] = 0.5* PI
                qpos[4] = PI*0.0    
        elif self.config.sensor_position == "Thumb_side":
            if(self.is_left):
                qpos[2] = 0.5* PI
                qpos[3] = -0.5* PI
                qpos[4] = -PI*0.5
            else:
                qpos[2] = -0.5* PI
                qpos[3] = 0.5* PI
                qpos[4] = PI*0.5   
        # 传感器在大拇指侧
        # if(self.is_left):
        #     qpos[2] = 0.5* PI
        #     qpos[3] = -0.5* PI
        #     qpos[4] = -PI*0.5
        # else:
        #     qpos[2] = -0.5* PI
        #     qpos[3] = 0.5* PI
        #     qpos[4] = PI*0.5
        self.qpos = copy.deepcopy(qpos)

        print(f"init qpos {self.is_left}: {qpos}\n")

        self.update_ik(qpos)
        self.tp = np.hstack((self.pos, self.eul))

        self.init_tp = np.hstack((self.pos, self.eul))

        print(f"init {self.init_tp }\n")

        self.init_rot = eul_to_rot(self.eul)



    def update_target_pos_diff(self, target_pos_eul_dff):
        if self.config.sensor_position == "Back_of_the_hand":
            if(self.is_left):
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]
                self.tp[1] = target_pos_eul_dff[0] + self.init_tp[1]
                self.tp[2] = -1*target_pos_eul_dff[1] + self.init_tp[2]
            else:
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]
                self.tp[1] = -1*target_pos_eul_dff[0] + self.init_tp[1]
                self.tp[2] = target_pos_eul_dff[1] + self.init_tp[2]
            deta_rot = eul_to_rot(target_pos_eul_dff[3:])
            tp_rot = self.init_rot @ deta_rot
            tp_eul = rot_to_eul(tp_rot)
            self.tp[3] =  tp_eul[0]
            self.tp[4] =  tp_eul[1]
            self.tp[5] =  tp_eul[2]
        elif self.config.sensor_position == "Palm_of_the_hand":
            if(self.is_left):
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]        # x 指示灯一侧 字为正
                self.tp[1] = -1*target_pos_eul_dff[0] + self.init_tp[1]        # y 指示灯侧面 左侧为正
                self.tp[2] = target_pos_eul_dff[1] + self.init_tp[2]     # z PICO字一侧 当前条件下 字侧为正
                # print(f"self.tp[0]: {self.tp[0]:.4g}, \tself.tp[1]: {self.tp[1]:.4g}, \tself.tp[2]: {self.tp[2]:.4g}")
            else:
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]
                self.tp[1] = target_pos_eul_dff[0] + self.init_tp[1]
                self.tp[2] = -1*target_pos_eul_dff[1] + self.init_tp[2]
            deta_rot = eul_to_rot(target_pos_eul_dff[3:])
            tp_rot = self.init_rot @ deta_rot
            tp_eul = rot_to_eul(tp_rot)
            self.tp[3] =  tp_eul[0]
            self.tp[4] =  tp_eul[1]
            self.tp[5] =  tp_eul[2]
        elif self.config.sensor_position == "Thumb_side":
            if(self.is_left):
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]        # x 指示灯一侧 字为正
                self.tp[1] = -1*target_pos_eul_dff[1] + self.init_tp[1]        # y 指示灯侧面 左侧为正
                self.tp[2] = -1*target_pos_eul_dff[0] + self.init_tp[2]     # z PICO字一侧 当前条件下 字侧为正
                # print(f"self.tp[0]: {self.tp[0]:.4g}, \tself.tp[1]: {self.tp[1]:.4g}, \tself.tp[2]: {self.tp[2]:.4g}")
            else:
                self.tp[0] = target_pos_eul_dff[2] + self.init_tp[0]    #
                self.tp[1] = -1*target_pos_eul_dff[1] + self.init_tp[1]    #
                self.tp[2] = -1*target_pos_eul_dff[0] + self.init_tp[2]    #
                # print(f"self.tp[0]: {self.tp[0]:.4g}, \tself.tp[1]: {self.tp[1]:.4g}, \tself.tp[2]: {self.tp[2]:.4g}")
            deta_rot = eul_to_rot(target_pos_eul_dff[3:])
            tp_rot = self.init_rot @ deta_rot
            tp_eul = rot_to_eul(tp_rot)
            self.tp[3] =  tp_eul[0]
            self.tp[4] =  tp_eul[1]
            self.tp[5] =  tp_eul[2]
        
        # if(self.is_left):
        #     log_m.real_pos_eul.update_log_info("INFO", "left pos eul" + str(self.pos.tolist() + self.eul.tolist()))
        # else:
        #     log_m.real_pos_eul.update_log_info("INFO", "right pos eul" + str(self.pos.tolist() + self.eul.tolist()))


    def update_ik(self, qpos):
        pinocchio.forwardKinematics(self.model, self.data, qpos)
        self.jacobian = pinocchio.computeJointJacobian(self.model, self.data, qpos, self.end_idx)
        # print(self.jacobian)
        # self.jacobian = pinocchio.computeFrameJacobian(self.model, self.data, qpos, self.end_idx)

        self.pos = self.data.oMi[self.end_idx].translation
        self.eul = rot_to_eul(self.data.oMi[self.end_idx].rotation)

        # self.elbow_pos = self.data.oMi[4].translation
        # self.elbow_eul = rot_to_eul(self.data.oMi[4].rotation)
        # print(self.jacobian)
        # if(self.is_left):
        #     print(f"left end pos : {self.pos}\n")
        # else:
        #     print(f"right end pos : {self.pos}\n")
        # # self.is_collapse()


    def is_collapse(self):
        inter_point = []
        for i in range(5):
            inter_point.append([(self.pos[0] - self.elbow_pos[0])*i/5.+self.elbow_pos[0], (self.pos[1] - self.elbow_pos[1])*i/5.+self.elbow_pos[1]])
        
        for point in inter_point:

            a = 0.22
            b = 0.22
            if((point[0]/a)**2 + (point[1]/b)**2 <1):
                print("COLLAPSE!!!!\n")
                print(f"is_left : {self.is_left} : {point}")
                import sys
                sys.exit()    

    def get_deta_h(self):
        t = self.qpos_threshold
        q = self.qpos
        a = 1
        b = 1
        for i in range(self.num_link):
            deta = abs(((t[i][1] - t[i][0]) ** 2 * (2 * q[i] - t[i][0] - t[i][1])) / (
                    4 * ((t[i][0] - q[i]) ** 2) * ((t[i][1] - q[i]) ** 2) + 1e-12))
            if (deta >= self.deta_h[i]):
                self.weight[i, i] = a + deta * b
            else:
                self.weight[i, i] = a

            self.weight[i, i] = min(1e20, self.weight[i, i])
            # self.deta_h[i] = deta

    def limit_qpos(self, theta):
        for i in range(self.num_link):
            if (theta[i] > self.qpos_threshold[i][1]):
                theta[i] = self.qpos_threshold[i][1]
            elif (theta[i] < self.qpos_threshold[i][0]):
                theta[i] = self.qpos_threshold[i][0]
        return theta



    def limit_vel(self, v):
        vel_ratio = 0.2
        acc_ratio = 0.5
        acc_threshold = 500
        max_vel = 100
        v_1 = [0 for _ in range(self.num_link)]
        for i in range(self.num_link):
            v_1[i] = vel_ratio * v[i] + (1 - vel_ratio) * self.vel[i]
            v_1[i] = min(max_vel, v_1[i])
        return np.array(v_1)
    


        acc = [0 for _ in range(self.num_link)]
        for i in range(self.num_link):
            acc[i] = v_1[i] - self.vel[i]
            if (acc[i] >= acc_threshold):
                acc[i] = acc_threshold
            elif (acc[i] < -acc_threshold):
                acc[i] = -acc_threshold

        for i in range(self.num_link):
            self.acc[i] = acc[i]
            self.vel[i] = self.acc[i] + self.vel[i]

        return np.array(self.vel)

    def inv_kinemic(self):
        dim = self.dim
        num_links = self.num_link

        theta = np.zeros([num_links], dtype=np.float32)

        damping = 100
        weight = np.identity(dim)

        ee = np.hstack((self.pos, self.eul))
        its = 0

        while True:

            tp = self.tp

            oMdes = pinocchio.SE3(eul_to_rot(tp[3:]), tp[:3])
            pinocchio.forwardKinematics(self.model, self.data, self.qpos)
            # iMd = self.data.oMi[self.num_link].actInv(oMdes)
            iMd = self.data.oMi[self.end_idx].actInv(oMdes)
            err = pinocchio.log(iMd).vector  # in joint frame
            err[3] = min(0.4, err[3])
            err[4] = min(0.4, err[4])
            err[5] = min(0.4, err[5])
            if norm(err) < eps or its % self.iter == 0:
                # print(f"update pos: {self.pos}")
                yield np.hstack((theta, np.zeros(1)))
            # err *= np.array((1, 1, 1, 0., 0., 0.))
            # err *= np.array((1, 1, 1, 0., 0., 0.))
            # err *= np.array((0, 0, 0, 1., 1., 1.))

            err *= np.array((1, 1, 1, 0.2, 0.2, 0.2))  # 正常运行时
            # err *= np.array((0, 0, 0, 0.2, 0.2, 0.2)) # 调试方向
            # err *= np.array((1, 1, 1, 0, 0., 0.)) # 调试位置


            J = pinocchio.computeJointJacobian(self.model, self.data, self.qpos, self.end_idx)  # in joint frame
            # J = pinocchio.computeFrameJacobian(self.model, self.data, self.qpos, self.end_idx)  # in joint frame

            J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)

            # lam = damp * np.eye(6)

            # v = - J.T.dot(solve(J.dot(J.T) + lam, err))
            self.get_deta_h()
            lam = damp * self.weight
            # print(f"weight : {self.weight}\n")

            v = -(np.linalg.pinv((J.T) @ J + lam)) @ J.T @ err

            v_output = self.limit_vel(v)
            # v_output = v

            theta = pinocchio.integrate(self.model, self.qpos, v_output * DT)
            theta = self.limit_qpos(theta)

            # print(f"theta {self.is_left}：{theta}\n")

            # print(f" theta : {theta}\n")
            self.qpos = theta
            self.update_ik(theta)


            its += 1
            # if(self.is_left):
            #     # print( [[self.tp.tolist()], [self.pos[0], self.pos[1], self.pos[2], self.eul[0], self.eul[1], self.eul[2]]])
            #     plot_util.plot_eul_list[0] = self.tp.tolist()
            #     plot_util.plot_eul_list[1] = [self.pos[0], self.pos[1], self.pos[2], self.eul[0], self.eul[1], self.eul[2]]
            # else:
            #     plot_util.plot_eul_list[2] = self.tp.tolist()
            #     plot_util.plot_eul_list[3] = [self.pos[0], self.pos[1], self.pos[2], self.eul[0], self.eul[1], self.eul[2]]    


    def wrist_control(self):
        limit = [[ -2.9147, 2.9147], [-0.9948, 0.9948], [-0.4538, 0.4538]]
        qpos = self.qpos[4:7]

        qpos[1] = self.eul[2]
        # qpos[2] = self.eul[2]
        if(self.is_left):
            print(f"eul ： {self.eul}\n")

        for i in range(3):
            qpos[i] = max(limit[i][0], qpos[i])
            qpos[i] = min(limit[i][1], qpos[i])
        return qpos

if __name__ == "__main__":
    s = loadUrdf()
    s.load_urdf()
