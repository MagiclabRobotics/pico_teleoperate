

from loguru import logger
import loguru
logger.remove(handler_id=None)  # 清除之前的设置

class logUnit():
    def __init__(self,name):
        self.name = name
        pass
    
    def update_log_info(self, log_type, info):
        info = self.name + " : " + info
        if(log_type=="WARNING"):
            logger.warning(info)
        elif(log_type=="INFO"):
            logger.info(info)
        elif(log_type=="ERROR"):
            logger.error(info)

    




class logManage():
    def __init__(self):
        logger.add('log/log_data/info.log', level='INFO', mode='w',format='{time} {level} {message}',rotation='00:00', retention='1 days', compression='zip', encoding='utf-8', enqueue=True)
        logger.add('log/log_data/error.log', level='ERROR', format='{time} {level} {message}',rotation='00:00', retention='1 days', compression='zip', encoding='utf-8', enqueue=True)
        logger.add('log/log_data/warning.log', level='WARNING', format='{time} {level} {message}',rotation='00:00', retention='1 days', compression='zip', encoding='utf-8', enqueue=True)

        self.tracker_state = logUnit("TRACKER STATE")
        self.mocap_info = logUnit("MOCAP INFO")
        self.target_pos_eul = logUnit("TARGET POS EUL INFO")
        self.real_pos_eul = logUnit("REAL POS EUL INFO")
        self.end_pos_error = logUnit("REAL POS EUL INFO")
        self.qpos_info = logUnit("REAL POS EUL INFO")


log_m = logManage()


if __name__ =="__main__":
    s = logManage()

    s.tracker_state.update_log_info("INFO","11111")
