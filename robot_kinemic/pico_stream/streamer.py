import grpc
from threading import Thread
import time 
import numpy as np 
from robot_kinemic.pico_stream.grpc_msg import * 
from robot_kinemic.kinemic_utils import *
# from robot_kinemic.log.log_manage import log_m


YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, 1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)

def process_matrix(message):
    m = np.array([[message.m00, message.m01, message.m02, message.m03],
                    [message.m10, message.m11, message.m12, message.m13],
                    [message.m20, message.m21, message.m22, message.m23],
                    [0, 0, 0, 1]])
    return m




class PicoxrControllerStreamer:

    def __init__(self, ip, port, record = True): 

        # Meta Quest IP 
        self.ip = ip
        self.port = port
        self.record = record 
        self.recording = [] 
        self.latest = None 
        self.axis_transform = YUP2ZUP
        
        
        self.l_btn_one = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.l_btn_two = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.l_trigger_index = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.l_trigger_hand = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.r_btn_one = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.r_btn_two = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.r_trigger_index = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.r_trigger_hand = xrTracking_pb2.ButtonState(state=0, message="idle")
        self.running = True
        # self.send_state()
        self.start_streaming()

    def start_streaming(self): 

        stream_thread = Thread(target = self.stream)
        stream_thread.start() 
        while self.latest is None and (self.running == True):
            time.sleep(0.001)
            pass 
        print(' == DATA IS FLOWING IN! ==')
        print('Ready to start streaming.') 

    def stop_streaming(self):
        self.running = False

    def stream(self): 

        # request_hand = xrtracking_pb2.HandUpdate()

        requst_controller = xrtracking_pb2.ControllerUpdate()
        try:
            with grpc.insecure_channel(f"{self.ip}:{self.port}") as channel:
                stub = xrtracking_pb2_grpc.TrackingServiceStub(channel)
                # responses = stub.StreamHandUpdates(request_hand)
                responses_v2 = stub.StreamButtonInfo(self.update_send_data())
                responses = stub.StreamControllerUpdates(requst_controller)
                # for response in responses_v2:
                #     print("Received from server:", response.code)

                for response in responses:

                    transformations = {
                        "head": process_matrix(response.Head),

                        "left_controller_matrix": process_matrix(response.left_controller.matrix),
                        "right_controller_matrix": process_matrix(response.right_controller.matrix),
                        "head": self.axis_transform @  process_matrix(response.Head),

                        "left_connect":response.left_controller.isConnected,
                        "right_connect":response.right_controller.isConnected,

                        "btn_l": [
                            response.left_controller.btn_one.isPressed, 
                            response.left_controller.btn_two.isPressed, 
                            response.left_controller.trigger_index.isPressed,  
                            response.left_controller.trigger_hand.isPressed, 
                            response.left_controller.btn_one.isTouched, 
                            response.left_controller.btn_two.isTouched, 
                            response.left_controller.trigger_index.isTouched, 
                            response.left_controller.trigger_hand.isTouched,
                            ],
                        
                        "btn_r": [
                            response.right_controller.btn_one.isPressed, 
                            response.right_controller.btn_two.isPressed, 
                            response.right_controller.trigger_index.isPressed,  
                            response.right_controller.trigger_hand.isPressed, 
                            response.right_controller.btn_one.isTouched, 
                            response.right_controller.btn_two.isTouched, 
                            response.right_controller.trigger_index.isTouched, 
                            response.right_controller.trigger_hand.isTouched,
                            ],

                        "r_axis_l": [
                            response.left_controller.thumb_stick.vector2.x, 
                            response.left_controller.thumb_stick.vector2.y],
                        "r_axis_r": [
                            response.right_controller.thumb_stick.vector2.x, 
                            response.right_controller.thumb_stick.vector2.y]
                    }


                    transformations.update({"tracker_list": [None, None, None]})

                    tracker_count = 0
                    for i, tracker in enumerate(response.motionTracker):

                        transformations["tracker_list"][i] = process_matrix(tracker.matrix)

                        tracker_count += 1

                    if(tracker_count !=3):
                        # log_m.tracker_state.update_log_info("ERROR", f"tracker count err : {tracker_count}\n")
                        # import sys
                        # print(f"tracker count err : {tracker_count}\n")
                        # sys.exit()
                        self.running = False
                        break

                    if self.record: 
                        self.recording.append(transformations)
                    self.latest = transformations 
                    if not (self.running is True):
                        break

        except Exception as e:
            print(f"An error occurred: {e}")
            self.running = False
            pass 

    def get_latest(self): 
        return self.latest
        
    def get_recording(self): 
        return self.recording
    
    def update_X(self, state, message):
        self.l_btn_one = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_Y(self, state, message):
        self.l_btn_two = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_l_index(self, state, message):
        self.l_trigger_index = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_l_hand(self, state, message):
        self.l_trigger_hand = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_A(self, state, message):
        self.r_btn_one = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_B(self, state, message):
        self.r_btn_two = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_r_index(self, state, message):
        self.r_trigger_index = xrTracking_pb2.ButtonState(state=state, message=message)
        
    def update_l_hand(self, state, message):
        self.r_trigger_hand = xrTracking_pb2.ButtonState(state=state, message=message)
    
    def update_send_data(self):
        while True:
            # self.random_update_value()
            message = xrTracking_pb2.ButtonInfo(
            l_btn_one=self.l_btn_one,
            l_btn_two=self.l_btn_two,
            l_trigger_index=self.l_trigger_index,
            l_trigger_hand=self.l_trigger_hand,
            
            r_btn_one=self.r_btn_one,
            r_btn_two=self.r_btn_two,
            r_trigger_index=self.r_trigger_index,
            r_trigger_hand=self.r_trigger_hand,
            )
            
            # print(f"Sending state: {message}")
            time.sleep(1)
            # return message
            yield message

    def send_state(self):
        with grpc.insecure_channel('192.168.12.181:12345') as channel:
            stub = xrTracking_pb2_grpc.TrackingServiceStub(channel)
            responses = stub.StreamButtonInfo(self.update_send_data())
            for response in responses:
                print("Received from server:", response.code)
    

if __name__ == "__main__": 

    streamer = MetaPlayControllerStreamer(ip = '10.29.230.57')
    while True: 

        latest = streamer.get_latest()
        print(latest)