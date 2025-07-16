


import copy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

from robot_kinemic.pico_stream import PicoxrControllerStreamer
from robot_kinemic.kinemic_utils import *
from robot_kinemic.conf import *
# from robot_kinemic.log.log_manage import log_m



def buildStreamer(_ip, _record):

    return PicoxrControllerStreamer(ip=_ip, port=12345, record=_record)

server_ip = "192.168.12.110"

class TeleopData():
    def __init__(self, server_ip):


        self.stream_contrller = buildStreamer(server_ip, False)
        self.mocap_map =  {"l_h":{"p_e": [0,0,0,0,0,0], "btn":[0,0,0,0], "r_axis":[0,0]},"r_h":{"p_e": [0,0,0,0,0,0], "btn":[0,0,0,0], "r_axis":[0,0]},"chest":{"p_e": [0,0,0,0,0,0]}, "head":{"p_e": [0,0,0,0,0,0]},"l_w":{"p_e": [0,0,0,0,0,0]}, "r_w":{"p_e": [0,0,0,0,0,0]}}


    def get_controller_data_l(self):
        info = self.stream_contrller.latest
        if(not info["left_connect"]):
            return None
        wrist_trans_l = info["left_controller_matrix"]
        pos_eul = np.zeros(6, dtype=float)
        pos_eul[:3] = wrist_trans_l[:3, 3]
        pos_eul[3:] = rot_to_eul(wrist_trans_l[:3,:3])
        btn = info["btn_l"]
        r_axis = info["r_axis_l"]
        mocap_info = {"p_e":pos_eul, "btn":btn, "r_axis":r_axis}
        return mocap_info


    def get_controller_data_r(self):
        info = self.stream_contrller.latest
        if(not info["right_connect"]):
            return None
        wrist_trans_r = info["right_controller_matrix"]
        pos_eul = np.zeros(6, dtype=float)
        pos_eul[:3] = wrist_trans_r[:3, 3]
        pos_eul[3:] = rot_to_eul(wrist_trans_r[:3,:3])
        btn = info["btn_r"]
        r_axis = info["r_axis_r"]
        mocap_info = {"p_e":pos_eul, "btn":btn, "r_axis":r_axis}
        return mocap_info


    
    def get_tracker_data(self):
        mocap_info = [None, None, None]
        info = self.stream_contrller.latest

        for i in range(3):
            tracker_info = info["tracker_list"][i]
            if(tracker_info is not None):
                pos_eul = np.zeros(6, dtype=float)
                pos_eul[:3] = tracker_info[:3, 3]
                pos_eul[3:] = rot_to_eul(tracker_info[:3,:3])
                mocap_info[i] = pos_eul

        return mocap_info

    def get_head_data(self):
        head = self.stream_contrller.latest["head"][0]

        pos_eul = np.zeros(6, dtype=float)
        pos_eul[:3] = head[:3, 3]
        pos_eul[3:] = rot_to_eul(head[:3,:3])
        mocap_info = {"head_p_e":pos_eul}

        return mocap_info
    



    def update_mocap_info(self):
        l_controller_info = self.get_controller_data_l()
        if(l_controller_info is not None):
            self.mocap_map.update({"l_h":l_controller_info})

        r_controller_info = self.get_controller_data_r()
        if(r_controller_info is not None):
            self.mocap_map.update({"r_h":r_controller_info})

        head_info = self.get_head_data()
        self.mocap_map.update({"chest":head_info})


        tracker_info = self.get_tracker_data()
        if(tracker_info[0] is not None):
            self.mocap_map["chest"]["p_e"] = tracker_info[0]
        if(tracker_info[1] is not None):
            self.mocap_map["l_w"]["p_e"] = tracker_info[1]
        if(tracker_info[2] is not None):
            self.mocap_map["r_w"]["p_e"] = tracker_info[2]

        # log_m.mocap_info.update_log_info("INFO", str(self.mocap_map))
        return self.mocap_map




if __name__ == "__main__":
    s = TeleopData(TeleopData_IP)

    while True:
        s.update_mocap_info()
