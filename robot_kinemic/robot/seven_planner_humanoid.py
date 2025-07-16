import numpy as np
# import matplotlib.pyplot as plt
import csv
import math

class SevenSegmentSpeedPlan:
    def __init__(self):
        self.MIN_VAL = 1e-7
        self.unispeed_time = 0

    def set_init_conditions(self, jerk_max, acc_max, speed_max, displacement):
        self.jerk_max = jerk_max
        self.acc_max = acc_max
        self.speed_max = speed_max
        self.displacement = displacement

    def seven_segment_speed_plan_func(self):
        if self.speed_max * self.jerk_max - self.acc_max * self.acc_max > self.MIN_VAL:
            self.accacc_time = self.acc_max / self.jerk_max
            self.decacc_time = self.accacc_time
            self.accdec_time = self.accacc_time
            self.decdec_time = self.accacc_time

            self.uniacc_time = self.speed_max / self.acc_max - self.acc_max / self.jerk_max
            self.unidec_time = self.uniacc_time
        else:
            self.accacc_time = np.sqrt(self.speed_max / self.jerk_max)
            self.decacc_time = self.accacc_time
            self.accdec_time = self.accacc_time
            self.decdec_time = self.accacc_time

            self.uniacc_time = 0
            self.unidec_time = self.uniacc_time

            self.acc_max = self.accacc_time * self.jerk_max

        self.acceleration_segment_disp = self.jerk_max * self.accacc_time * (self.accacc_time + self.uniacc_time) * (0.5 * self.uniacc_time + self.accacc_time)
        self.deceleration_segment_disp = self.acceleration_segment_disp
        self.unispeed_Disp = self.displacement - self.acceleration_segment_disp - self.deceleration_segment_disp

        if self.unispeed_Disp > self.MIN_VAL:
            self.unispeed_time = self.unispeed_Disp / self.speed_max
        else:
            self.unispeed_Disp = 0
            if (self.displacement - 2 * (self.acc_max ** 3) / (self.jerk_max ** 2)) > self.MIN_VAL:
                self.accacc_time = self.acc_max / self.jerk_max
                self.decacc_time = self.accacc_time
                self.accdec_time = self.accacc_time
                self.decdec_time = self.accacc_time

                self.uniacc_time = -1.5 * self.accacc_time + np.sqrt((self.accacc_time / 2) ** 2 + self.displacement / self.acc_max)
                self.unidec_time = self.uniacc_time
                self.unispeed_time = 0
            else:
                self.accacc_time = (self.displacement / (2 * self.jerk_max)) ** (1 / 3)
                self.decacc_time = self.accacc_time
                self.accdec_time = self.accacc_time
                self.decdec_time = self.accacc_time

                self.uniacc_time = 0
                self.unidec_time = 0
                self.unispeed_time = 0
                self.acc_max = self.jerk_max * self.accacc_time

        self.time_length = self.accacc_time + self.uniacc_time + self.decacc_time + self.unispeed_time + self.accdec_time + self.unidec_time + self.decdec_time

    def cal_accacc_segment_end_data(self):
        self.accacc_Jerk = self.jerk_max
        self.accacc_Acc = self.jerk_max * self.accacc_time
        self.accacc_Speed = 0.5 * self.jerk_max * self.accacc_time ** 2
        self.accacc_Disp = self.jerk_max * self.accacc_time ** 3 / 6

    def cal_accacc_segment_data(self, time):
        self.cur_jerk = self.jerk_max
        self.cur_acc = self.jerk_max * time
        self.cur_speed = 0.5 * self.jerk_max * time ** 2
        self.cur_disp = self.jerk_max * time ** 3 / 6
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_uniacc_segment_end_data(self):
        self.uniacc_Jerk = 0
        self.uniacc_Acc = self.accacc_Acc
        self.uniacc_Speed = self.accacc_Speed + self.accacc_Acc * self.uniacc_time
        self.uniacc_Disp = self.accacc_Speed * self.uniacc_time + 0.5 * self.accacc_Acc * self.uniacc_time ** 2

    def cal_uniacc_segment_data(self, time):
        self.cur_jerk = 0
        self.cur_acc = self.accacc_Acc
        self.cur_speed = self.accacc_Speed + self.accacc_Acc * time
        self.cur_disp = self.accacc_Disp + self.accacc_Speed * time + 0.5 * self.accacc_Acc * time ** 2
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_decacc_segment_end_data(self):
        self.decacc_Jerk = -self.jerk_max
        self.decacc_Acc = self.uniacc_Acc - self.jerk_max * self.decacc_time
        self.decacc_Speed = self.uniacc_Speed + self.uniacc_Acc * self.decacc_time - 0.5 * self.jerk_max * self.decacc_time ** 2
        self.decacc_Disp = self.uniacc_Speed * self.decacc_time + 0.5 * self.uniacc_Acc * self.decacc_time ** 2 - self.jerk_max * self.decacc_time ** 3 / 6
        self.acceleration_segment_disp = self.decacc_Disp + self.uniacc_Disp + self.accacc_Disp
        self.acceleration_segment_time = self.decacc_time + self.uniacc_time + self.accacc_time

    def cal_decacc_segment_data(self, time):
        self.cur_jerk = -self.jerk_max
        self.cur_acc = self.uniacc_Acc - self.jerk_max * time
        self.cur_speed = self.uniacc_Speed + self.uniacc_Acc * time - 0.5 * self.jerk_max * time ** 2
        self.cur_disp = self.accacc_Disp + self.uniacc_Disp + self.uniacc_Speed * time + 0.5 * self.uniacc_Acc * time ** 2 - self.jerk_max * time ** 3 / 6
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_unispeed_segment_end_data(self):
        self.unispeed_Jerk = 0
        self.unispeed_Acc = 0
        self.unispeed_Speed = self.decacc_Speed
        self.unispeed_Disp = self.unispeed_Speed * self.unispeed_time

    def cal_unispeed_segment_data(self, time):
        self.cur_jerk = 0
        self.cur_acc = 0
        self.cur_speed = self.decacc_Speed
        self.cur_disp = self.acceleration_segment_disp + self.unispeed_Speed * time
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_accdec_segment_end_data(self):
        self.accdec_Jerk = -self.jerk_max
        self.accdec_Acc = -self.jerk_max * self.accdec_time
        self.accdec_Speed = self.unispeed_Speed - 0.5 * self.jerk_max * self.accdec_time ** 2
        self.accdec_Disp = self.unispeed_Speed * self.accdec_time - self.jerk_max * self.accdec_time ** 3 / 6

    def cal_accdec_segment_data(self, time):
        self.cur_jerk = -self.jerk_max
        self.cur_acc = -self.jerk_max * time
        self.cur_speed = self.unispeed_Speed - 0.5 * self.jerk_max * time ** 2
        self.cur_disp = self.acceleration_segment_disp + self.unispeed_Disp + self.unispeed_Speed * time - self.jerk_max * time ** 3 / 6
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_unidec_segment_end_data(self):
        self.unidec_Jerk = 0
        self.unidec_Acc = self.accdec_Acc
        self.unidec_Speed = self.accdec_Speed + self.accdec_Acc * self.unidec_time
        self.unidec_Disp = self.accdec_Speed * self.unidec_time + 0.5 * self.accdec_Acc * self.unidec_time ** 2

    def cal_unidec_segment_data(self, time):
        self.cur_jerk = 0
        self.cur_acc = self.accdec_Acc
        self.cur_speed = self.accdec_Speed + self.accdec_Acc * time
        self.cur_disp = self.acceleration_segment_disp + self.unispeed_Disp + self.accdec_Disp + self.accdec_Speed * time + 0.5 * self.accdec_Acc * time ** 2
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_decdec_segment_end_data(self):
        self.decdec_Jerk = self.jerk_max
        self.decdec_Acc = 0
        self.decdec_Speed = 0
        self.decdec_Disp = self.unidec_Speed * self.decdec_time + 0.5 * self.unidec_Acc * self.decdec_time ** 2 - self.jerk_max * self.decdec_time ** 3 / 6
        self.deceleration_segment_disp = self.accdec_Disp + self.unidec_Disp + self.decdec_Disp

    def cal_decdec_segment_data(self, time):
        self.cur_jerk = self.jerk_max
        self.cur_acc = self.unidec_Acc + self.jerk_max * time
        self.cur_speed = self.unidec_Speed + self.unidec_Acc * time + 0.5 * self.jerk_max * time ** 2
        self.cur_disp = self.acceleration_segment_disp + self.unispeed_Disp + self.accdec_Disp + self.unidec_Disp + self.unidec_Speed * time + 0.5 * self.unidec_Acc * time ** 2 + self.jerk_max * time ** 3 / 6
        self.cur_disp_normalization_ratio = self.cur_disp / self.displacement

    def cal_total_segment_end_data(self):
        self.total_Disp = self.acceleration_segment_disp + self.unispeed_Disp + self.deceleration_segment_disp

    def cal_max_Acc_Speed(self):
        self.actual_max_acc = self.uniacc_Acc
        self.actual_max_speed = self.decacc_Speed
    def create_time_data(self, num):
        self.cal_accacc_segment_end_data()
        self.cal_uniacc_segment_end_data()
        self.cal_decacc_segment_end_data()
        self.cal_unispeed_segment_end_data()
        self.cal_accdec_segment_end_data()
        self.cal_unidec_segment_end_data()
        self.cal_decdec_segment_end_data()
        self.cal_total_segment_end_data()
        self.cal_max_Acc_Speed()

        self.cur_jerk_list = []
        self.cur_acc_list = []
        self.cur_speed_list = []
        self.cur_disp_list = []
        self.cur_disp_normalization_ratio_list = []

        self.time_step = self.time_length / (num - 1)
        # self.time_step = 0.002
        # num = int(self.time_length*500)
        # print(f"time_length: {self.time_length} num: {num}")
        
        for idx in range(num):
            time = self.time_step * idx

            if time < self.accacc_time:
                self.cal_accacc_segment_data(time)
            elif time < self.accacc_time + self.uniacc_time:
                self.cal_uniacc_segment_data(time - self.accacc_time)
            elif time < self.accacc_time + self.uniacc_time + self.decacc_time:
                self.cal_decacc_segment_data(time - self.accacc_time - self.uniacc_time)
            elif time < self.acceleration_segment_time + self.unispeed_time:
                self.cal_unispeed_segment_data(time - self.acceleration_segment_time)
            elif time < self.acceleration_segment_time + self.unispeed_time + self.accdec_time:
                self.cal_accdec_segment_data(time - self.acceleration_segment_time - self.unispeed_time)
            elif time < self.acceleration_segment_time + self.unispeed_time + self.accdec_time + self.unidec_time:
                self.cal_unidec_segment_data(time - self.acceleration_segment_time - self.unispeed_time - self.accdec_time)
            else:
                self.cal_decdec_segment_data(time - self.acceleration_segment_time - self.unispeed_time - self.accdec_time - self.unidec_time)


            self.cur_disp = self.cur_disp
            self.cur_jerk_list.append(self.cur_jerk)
            self.cur_acc_list.append(self.cur_acc)
            self.cur_speed_list.append(self.cur_speed)
            self.cur_disp_list.append(self.cur_disp)
            self.cur_disp_normalization_ratio_list.append(self.cur_disp_normalization_ratio)
        
        
    def create_time_data_auto_num(self, time_step=0.01):
        self.cal_accacc_segment_end_data()
        self.cal_uniacc_segment_end_data()
        self.cal_decacc_segment_end_data()
        self.cal_unispeed_segment_end_data()
        self.cal_accdec_segment_end_data()
        self.cal_unidec_segment_end_data()
        self.cal_decdec_segment_end_data()
        self.cal_total_segment_end_data()
        self.cal_max_Acc_Speed()

        self.cur_jerk_list = []
        self.cur_acc_list = []
        self.cur_speed_list = []
        self.cur_disp_list = []
        self.cur_disp_normalization_ratio_list = []
        # self.time_step = self.time_length / (num - 1)
        # self.time_step = 0.002
        time_step = 0.004
        self.time_step = time_step
        num = int(self.time_length* 1/self.time_step)
        # print(f"time_length: {self.time_length} num: {num}")
        
        for idx in range(num):
            time = self.time_step * idx

            if time < self.accacc_time:
                self.cal_accacc_segment_data(time)
            elif time < self.accacc_time + self.uniacc_time:
                self.cal_uniacc_segment_data(time - self.accacc_time)
            elif time < self.accacc_time + self.uniacc_time + self.decacc_time:
                self.cal_decacc_segment_data(time - self.accacc_time - self.uniacc_time)
            elif time < self.acceleration_segment_time + self.unispeed_time:
                self.cal_unispeed_segment_data(time - self.acceleration_segment_time)
            elif time < self.acceleration_segment_time + self.unispeed_time + self.accdec_time:
                self.cal_accdec_segment_data(time - self.acceleration_segment_time - self.unispeed_time)
            elif time < self.acceleration_segment_time + self.unispeed_time + self.accdec_time + self.unidec_time:
                self.cal_unidec_segment_data(time - self.acceleration_segment_time - self.unispeed_time - self.accdec_time)
            else:
                self.cal_decdec_segment_data(time - self.acceleration_segment_time - self.unispeed_time - self.accdec_time - self.unidec_time)
            self.cur_disp = self.cur_disp
            self.cur_jerk_list.append(self.cur_jerk)
            self.cur_acc_list.append(self.cur_acc)
            self.cur_speed_list.append(self.cur_speed)
            self.cur_disp_list.append(self.cur_disp)
            self.cur_disp_normalization_ratio_list.append(self.cur_disp_normalization_ratio)

    # def plot_kinematic_data(self):
    #     time_vector = np.linspace(0, self.time_length, len(self.cur_speed_list))

    #     plt.figure(figsize=(10, 15))

    #     plt.subplot(4, 1, 1)
    #     plt.plot(time_vector, self.cur_jerk_list)
    #     plt.ylabel("Jerk (m/s^3)")
    #     plt.title("Jerk vs Time")

    #     plt.subplot(4, 1, 2)
    #     plt.plot(time_vector, self.cur_acc_list)
    #     plt.ylabel("Acceleration (m/s^2)")
    #     plt.title("Acceleration vs Time")

    #     plt.subplot(4, 1, 3)
    #     plt.plot(time_vector, self.cur_speed_list)
    #     plt.ylabel("Speed (m/s)")
    #     plt.title("Speed vs Time")

    #     plt.subplot(4, 1, 4)
    #     plt.plot(time_vector, self.cur_disp_list)
    #     plt.xlabel("Time (s)")
    #     plt.ylabel("Displacement (m)")
    #     plt.title("Displacement vs Time")

    #     plt.tight_layout()
    #     plt.show()

    def save_data_to_csv(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['Time', 'Jerk', 'Acceleration', 'Speed', 'Displacement', 'Normalization Ratio']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            time_vector = np.linspace(0, self.time_length, len(self.cur_speed_list))

            for idx in range(len(time_vector)):
                writer.writerow({
                    'Time': time_vector[idx],
                    'Jerk': self.cur_jerk_list[idx],
                    'Acceleration': self.cur_acc_list[idx],
                    'Speed': self.cur_speed_list[idx],
                    'Displacement': self.cur_disp_list[idx],
                    'Normalization Ratio': self.cur_disp_normalization_ratio_list[idx]
                })
                
    def seven_segment_speed_plan(self, jerk_max=2.356194490192345, acc_max=1.570796326794897, speed_max=45/180*math.pi, displacement=3.979350694547072, time_step=0.01):
        self.set_init_conditions(jerk_max=jerk_max, acc_max=acc_max, speed_max=speed_max, displacement=displacement)
        self.seven_segment_speed_plan_func()        
        self.create_time_data_auto_num(time_step)
        # self.plot_kinematic_data()
        return self.cur_disp_list


def get_seven_segment_speed_plan(jerk_max=2.356194490192345, acc_max=1.570796326794897, speed_max=45/180*math.pi, displacement=3.979350694547072, time_step=0.01):
    planner = SevenSegmentSpeedPlan()
    planner.set_init_conditions(jerk_max=jerk_max, acc_max=acc_max, speed_max=speed_max, displacement=displacement)
    planner.seven_segment_speed_plan_func()
    # planner.create_time_data_auto_num(jerk_max, acc_max, speed_max, displacement, time_step)
    planner.create_time_data_auto_num(time_step)
    # planner.plot_kinematic_data()
    return planner.cur_disp_list


InterpolationSize = 100

class RobotPlaner(object):
    def __init__(self):
        # fully_open
        self.hand_home_pos = np.array([165, 176, 176, 176, 25.0, 165.0, 165, 176, 176, 176, 25.0, 165.0],dtype=np.float64)
        # pre_grasp
        # self.hand_home_pos = np.array([176, 176, 176, 176, 53.0, 90.0, 176, 176, 176, 176, 53.0, 90.0],dtype=np.float64)
        # grasp
        # self.hand_home_pos = np.array([114, 114, 113, 114, 53.0, 90.0, 114, 114, 113, 114, 53.0, 90.0],dtype=np.float64)

        # self.hand_home_pos = np.array([0,0,0,0,0,0,
        #                                0,0,0,0,0,0], 
        #                                 dtype=np.float64)
        # self.hand_home_pos = np.array([90,90,90,90,90,90,
        #                                90,90,90,90,90,90], 
        #                                 dtype=np.float64)
        self.interpolation_size = InterpolationSize + 1
        self.interpolation_size_min = InterpolationSize + 1
        self.interpolation_size_max = InterpolationSize + 1

    def set_interpolation_size(self, p_size):
        self.interpolation_size = p_size + 1
        self.interpolation_size_min = p_size + 1
        self.interpolation_size_max = p_size + 1

    def non_linear_space(self, start, end, num_points, a=3.0):
        """
        Generate a non-linearly spaced array of values using a Sigmoid function.

        :param start: Start value of the range
        :param end: End value of the range
        :param num_points: Number of values to generate
        :param a: Controls the steepness of the Sigmoid function
        :return: A non-linearly spaced array
        """
        res = np.array([])
        start_len = len(start)
        end_len = len(end)
        if start_len != end_len:
            return res
        for i in range(start_len):
            # 生成等间隔的线性空间数组
            start_i = start[i]
            end_i = end[i]
            # 使用Sigmoid函数来调整分布
            # linear_space = np.linspace(-a, a, num_points)
            # sigmoid_space = 1 / (1 + np.exp(-linear_space))
            
            linear_space = np.linspace( math.pi, 2*math.pi, num_points)
            sigmoid_space = (1 + np.cos(linear_space))/2
            # 缩放sigmoid_space以匹配指定的start和end范围
            res_tmp = start_i + (end_i - start_i) * sigmoid_space
            # print(res.shape)
            # print(res.shape)
            if res.shape == (0,):
                res = res_tmp
            elif res.shape == (num_points,):
                res = np.concatenate((res[:, np.newaxis], res_tmp[:, np.newaxis]), axis=1)
            else:
                res = np.concatenate((res, res_tmp[:, np.newaxis]), axis=1)
        # 返回两端密集中间稀疏的值
        return res
        
    def joint_pos_a2b(self, start, end):
        interpolate_demo_trajectory = None
        try:
            last_joints = start
            start = np.array(last_joints)
            stop = np.array(end)
            err = 0
            for i in range(14):
                err += abs(end[i]-start[i])
            if err>10:
                interpolate_demo_trajectory = np.linspace(start, stop, num=self.interpolation_size_max, axis=1).transpose()[1:self.interpolation_size_max, :]
            elif err<1:
                interpolate_demo_trajectory = np.linspace(start, stop, num=self.interpolation_size_min, axis=1).transpose()[1:self.interpolation_size_min, :]
            else:
                interpolate_demo_trajectory = np.linspace(start, stop, num=self.interpolation_size, axis=1).transpose()[1:self.interpolation_size, :]
            hand_home_pos = self.hand_home_pos * 3.1415926 / 180.0
            hand_home_pos = np.expand_dims(hand_home_pos, 0).repeat(interpolate_demo_trajectory.shape[0], axis=0)
            interpolate_demo_trajectory = np.concatenate((interpolate_demo_trajectory, hand_home_pos), axis=1)

            waist_zero = np.array([math.pi/6, math.pi/6], dtype=np.float64)
            # waist_zero = np.array([0, 0], dtype=np.float64)
            waist_expand = np.expand_dims(waist_zero, 0).repeat(interpolate_demo_trajectory.shape[0], axis=0)
            interpolate_demo_trajectory = np.concatenate((interpolate_demo_trajectory, waist_expand), axis=1)
            
            head_zero = np.array([0, 0], dtype=np.float64)
            head_expand = np.expand_dims(head_zero, 0).repeat(interpolate_demo_trajectory.shape[0], axis=0)
            interpolate_demo_trajectory = np.concatenate((interpolate_demo_trajectory, head_expand), axis=1)

            # print(interpolate_demo_trajectory.shape)
        except Exception as e:
            print(e)
        return interpolate_demo_trajectory

    def joint_pos_a2b_non_linear(self, start, end):
        interpolate_demo_trajectory = None
        try:
            last_joints = start
            start = np.array(last_joints)
            stop = np.array(end)
            err = 0
            for i in range(14):
                err += abs(end[i]-start[i])
            if err>10:
                interpolate_demo_trajectory = self.non_linear_space(start, stop, self.interpolation_size_max, 10.0)
            elif err<1:
                interpolate_demo_trajectory = self.non_linear_space(start, stop, self.interpolation_size_max, 10.0)
            else:
                interpolate_demo_trajectory = self.non_linear_space(start, stop, self.interpolation_size_max, 10.0)
        except Exception as e:
            print(e)
        return interpolate_demo_trajectory
    
    def joint_pos_a2b_seven_segment_speed_planer(self, start, end, speed=45/180*math.pi, time_step=0.01):
        interpolate_demo_trajectory = None
        try:
            last_joints = start
            start = np.array(last_joints)
            stop = np.array(end)
            
            jerk = 2.356194490192345
            acc = 1.570796326794897 / 3
            # speed = 45/180*math.pi
            pos_len = len(start)
            delta_max = 0
            delta_list = []
            hand_id_list = list(range(14,26))
            for i in range(pos_len):
                if i in hand_id_list:
                    continue
                si = start[i]
                ei = end[i]
                delta = ei-si
                delta_list.append(delta)
                delta_abs = abs(delta)
                if delta_max < delta_abs:
                    delta_max = delta_abs
            if delta_max == 0:
                return res
            res = get_seven_segment_speed_plan(jerk, acc, speed, delta_max, time_step)
            res = np.array(res)
            res = res/res.max()
            joint_pos_list = []
            j_res = None
            for i in range(pos_len):
                si = start[i]
                ei = stop[i]
                delta = ei-si
                delta_abs = abs(delta)
                if delta < 0:
                    j_res = si-res*delta_abs
                else:
                    j_res = si+res*delta_abs
                joint_pos_list.append(j_res)
            joint_pos_list = np.array(joint_pos_list)
            joint_pos_list = joint_pos_list.T
            interpolate_demo_trajectory = joint_pos_list
        except Exception as e:
            print(e)
        return interpolate_demo_trajectory


    def joint_pos_a_via_b_to_c(self, a, b, c):
        interpolate_trajectory = None
        try:
            interpolate_trajectory1 = self.joint_pos_a2b(a, b)
            interpolate_trajectory2 = self.joint_pos_a2b(b, c)
            interpolate_trajectory = np.concatenate((interpolate_trajectory1, interpolate_trajectory2), axis=0)
        except Exception as e:
            print(e)
        return interpolate_trajectory


    def joint_pos_a_via_fb_to_c(self, a, c):
        '''
            fix waypoint
        '''
        waypoint = [0, -1.5708, 0, 0, 0, 0, 0,
                    0, -1.5708, 0, 0, 0, 0, 0]
        interpolate_trajectory = None
        try:
            interpolate_trajectory1 = self.joint_pos_a2b(a, waypoint)
            interpolate_trajectory2 = self.joint_pos_a2b(waypoint, c)
            interpolate_trajectory = np.concatenate((interpolate_trajectory1, interpolate_trajectory2), axis=0)
        except Exception as e:
            print(e)
        return interpolate_trajectory

    def joint_pos_via_fb_list(self, pos_list):
        '''
            fix waypoint
            pos_list min size: 2
        '''
        waypoint = [0, 1.5708, 0, 0, 0, 0, 0,
                    0, -1.5708, 0, 0, 0, 0, 0]
        interpolate_trajectory = None
        try:
            len_pos = len(pos_list)
            if(len_pos<2):
                return interpolate_trajectory 
            for i in range(len_pos-1):
                pos_1 = pos_list[i]
                pos_2 = pos_list[i+1]
                interpolate_trajectory1 = self.joint_pos_a2b(pos_1, waypoint)
                interpolate_trajectory2 = self.joint_pos_a2b(waypoint, pos_2)
                interpolate_trajectory_t = np.concatenate((interpolate_trajectory1, interpolate_trajectory2), axis=0)
                if interpolate_trajectory is None:
                    interpolate_trajectory = interpolate_trajectory_t
                else:
                    interpolate_trajectory = np.concatenate((interpolate_trajectory, interpolate_trajectory_t), axis=0)

        except Exception as e:
            print(e)
        return interpolate_trajectory

    def joint_pos_ab_list(self, pos_list):
        '''
            pos_list min size: 2
        '''
        interpolate_trajectory = None
        try:
            len_pos = len(pos_list)
            if(len_pos<2):
                return interpolate_trajectory 
            for i in range(len_pos-1):
                pos_1 = pos_list[i]
                pos_2 = pos_list[i+1]
                interpolate_trajectory_t = self.joint_pos_a2b(pos_1, pos_2)
                if interpolate_trajectory is None:
                    interpolate_trajectory = interpolate_trajectory_t
                else:
                    interpolate_trajectory = np.concatenate((interpolate_trajectory, interpolate_trajectory_t), axis=0)
        except Exception as e:
            print(e)
        return interpolate_trajectory

    def joint_pos_ab_list_non_linear(self, pos_list):
        '''
            pos_list min size: 2
        '''
        interpolate_trajectory = None
        try:
            len_pos = len(pos_list)
            if(len_pos<2):
                return interpolate_trajectory 
            for i in range(len_pos-1):
                pos_1 = pos_list[i]
                pos_2 = pos_list[i+1]
                interpolate_trajectory_t = self.joint_pos_a2b_non_linear(pos_1, pos_2)
                if interpolate_trajectory is None:
                    interpolate_trajectory = interpolate_trajectory_t
                else:
                    interpolate_trajectory = np.concatenate((interpolate_trajectory, interpolate_trajectory_t), axis=0)
        except Exception as e:
            print(e)
        return interpolate_trajectory
    
    def joint_pos_ab_list_seven_segment(self, pos_list, speed=45/180*math.pi, time_step=0.01):
        '''
            pos_list min size: 2
        '''
        interpolate_trajectory = None
        try:
            len_pos = len(pos_list)
            if(len_pos<2):
                return interpolate_trajectory 
            for i in range(len_pos-1):
                pos_1 = pos_list[i]
                pos_2 = pos_list[i+1]
                interpolate_trajectory_t = self.joint_pos_a2b_seven_segment_speed_planer(pos_1, pos_2, speed, time_step)
                if interpolate_trajectory is None:
                    interpolate_trajectory = interpolate_trajectory_t
                else:
                    interpolate_trajectory = np.concatenate((interpolate_trajectory, interpolate_trajectory_t), axis=0)
        except Exception as e:
            print(e)
        return interpolate_trajectory


def get_pos_list(start, end, T=100):
    try:
        N=0
        pos_list = []
        pos_list.append(start)
        pos_list.append(end)
        planer = RobotPlaner()
        planer.set_interpolation_size(T)
        interpolate_trajectory = planer.joint_pos_ab_list_non_linear(pos_list)
        N = interpolate_trajectory.shape[0]
        return interpolate_trajectory, N
    except Exception as e:
        print("e")
        return None, 0


def get_pos_list_seven_segment(start, end, speed=45/180*math.pi, time_step=0.01):
    try:
        N=0
        pos_list = []
        pos_list.append(start)
        pos_list.append(end)
        planer = RobotPlaner()
        # planer.set_interpolation_size(T)
        interpolate_trajectory = planer.joint_pos_ab_list_seven_segment(pos_list, speed, time_step)
        N = interpolate_trajectory.shape[0]
        return interpolate_trajectory, N
    except Exception as e:
        print("e")
        return None, 0
    
    
    