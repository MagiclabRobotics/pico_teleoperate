import configparser

def parse_bool(value):
    return value.lower() in ('True','true', 'yes', '1', 'on')

class Config():
    def __init__(self):
        self.is_real = False
        self.is_gripper = False
        self.teleop_data_ip = False
        self.sensor_position = False
        
        self.record_csv = False
        self.save_path = False
        
        self.arm_l = [[-1.8,0.5], [0, 2.5656], [-2., 2.], [-2.2166, -0.], [ -2.9147, 2.9147], [0, 0], [0, 0]]
        self.arm_r = [[-1.8,0.5], [-2.5656, 0], [-2., 2.], [0., 2.2166], [ -2.9147, 2.9147], [0, 0], [0, 0]]
        self.hand_l = [[0.5, 3.0], [0.5, 3.0], [0.5, 3.0], [0.5, 3.0], [0.5, 0.92], [0.77, 2.87]]
        self.hand_r = [[0.5, 3.0], [0.5, 3.0], [0.5, 3.0], [0.5, 3.0], [0.5, 0.92], [0.77, 2.87]]
        self.perl = [[-0.2618, 0.2618], [0.0, 0.5], [-1.5708, 1.5708]]
        self.head = [[-0.5236, 0.5236], [-0.3491, 0.1745]]
        self.wrist_l = [[-0.9948, 0.9948], [-0.4538, 0.4538]]
        self.wrist_r = [[-0.9948, 0.9948], [-0.4538, 0.4538]]
        self.gripper_l = [[0.0, 80.0]]
        self.gripper_r = [[0.0, 80.0]]
        
    def multiline(sefl, config, title, key):
        multiline = config[title].get(key, '')
        value = [line.strip() for line in multiline.strip().splitlines() if line.strip()]
        res = []
        for v in value:
            data = [float(d) for d in v.split(',') ]
            res.append(data)
        return res
        
    def read_config_file(self, config_file):
        config = configparser.ConfigParser()
        config.read(config_file, encoding='utf-8')
        # setting
        self.is_real = parse_bool(config['settings'].get('is_real', "false"))
        self.is_gripper = parse_bool(config['settings'].get('is_gripper', "false"))
        self.teleop_data_ip = config['settings'].get('teleop_data_ip', "")
        self.sensor_position = config['settings'].get('sensor_position', "")
        # data_csv 
        self.record_csv = parse_bool(config['data_csv'].get('record_csv', "false"))
        self.save_path = config['data_csv'].get('save_path', "")
        # limit
        self.arm_l = self.multiline(config, "limit", "arm_l")
        self.arm_r = self.multiline(config, "limit", "arm_r")
        self.hand_l = self.multiline(config, "limit", "hand_l")
        self.hand_r = self.multiline(config, "limit", "hand_r")
        self.perl = self.multiline(config, "limit", "perl")
        self.head = self.multiline(config, "limit", "head")
        self.wrist_l = self.multiline(config, "limit", "wrist_l")
        self.wrist_r = self.multiline(config, "limit", "wrist_r")
        self.gripper_l = self.multiline(config, "limit", "gripper_l")
        self.gripper_r = self.multiline(config, "limit", "gripper_r")




if __name__ == "__main__":
    file_path = "./robot_kinemic.ini"
    # res = read_ini_multiline_list(file_path)
    handler = Config()
    handler.read_config_file(file_path)
    print("-------------------------------------------")
    print(f"is_real: {handler.is_real}", "")
    print(f"is_gripper: {handler.is_gripper}", "")
    print(f"teleop_data_ip: {handler.teleop_data_ip}", "")
    print(f"sensor_position: {handler.sensor_position}", "")    
    print(f"record_csv: {handler.record_csv}", "")
    print(f"save_path: {handler.save_path}", "")
    print(f"arm_l: {handler.arm_l}")
    print(f"arm_r: {handler.arm_r}")
    print(f"hand_l: {handler.hand_l}")
    print(f"hand_r: {handler.hand_r}")
    print(f"perl: {handler.perl}")
    print(f"head: {handler.head}")
    print(f"wrist_l: {handler.wrist_l}")
    print(f"wrist_r: {handler.wrist_r}")
    print(f"gripper_l: {handler.gripper_l}")
    print(f"gripper_r: {handler.gripper_r}")