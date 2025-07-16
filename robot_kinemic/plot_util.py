import sys
import time
from datetime import datetime
import numpy as np
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QApplication
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
from PyQt5.QtCore import QThread, pyqtSignal
from matplotlib import pyplot as plt
import threading
 
plot_eul_list = [None for _ in range(10)]
timeX = []
class Matplotib(QWidget):
    def __init__(self):
        super(Matplotib, self).__init__()
        self.initData()
 
    def initData(self):
        self.resize(1300, 800)
        # plt.rcParams['font.sans-serif'] = ['SimHei']
        # plt.rcParams['axes.unicode_minus'] = False
        # 设置画布部分
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.myCanvas = FC(self.fig)
        # 设置布局，将组件添加到布局中
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.myCanvas)
        self.setLayout(self.layout)
        # 创建线程并连接信号
        self.myThread = MyThread()
        self.myThread.update_data.connect(self.updateData)
        self.myThread.start()
 

    def updateData(self):
        plt.cla()  # 清空画布
        global plot_eul_list
        eul_list = plot_eul_list

        ax = self.ax
        num_eul = len(eul_list)
        for i in range(num_eul):
            if(eul_list[i] is None):
                continue

            if(len(eul_list[i])==3):
                roll = eul_list[i][0]
                pitch = eul_list[i][1]
                yaw = eul_list[i][2]
                px=1
                py=1
                pz=1
            elif(len(eul_list[i])==6):
                px=eul_list[i][0]
                py=eul_list[i][1]
                pz=eul_list[i][2]
                roll = eul_list[i][3]
                pitch = eul_list[i][4]
                yaw = eul_list[i][5]


            # Convert Euler angles to rotation matrix
            R_x = np.array([[1, 0, 0],
                            [0, np.cos(roll), -np.sin(roll)],
                            [0, np.sin(roll), np.cos(roll)]])
        
            R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        
            R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0],
                            [0, 0, 1]])
        
            R = np.dot(R_z, np.dot(R_y, R_x))
        

        
            # Plot the coordinate system
            origin = np.array([px, py, pz])
            x_axis = np.array([0.3, 0, 0])
            y_axis = np.array([0, 0.3, 0])
            z_axis = np.array([0, 0, 0.3])
        
            # ax.quiver(*origin, *x_axis, color='r')
            # ax.quiver(*origin, *y_axis, color='g')
            # ax.quiver(*origin, *z_axis, color='b')
        
            # Plot the transformed coordinate system
            transformed_x = np.dot(R, x_axis)
            transformed_y = np.dot(R, y_axis)
            transformed_z = np.dot(R, z_axis)
        
            ax.quiver(*origin, *transformed_x, color='r', linestyle='dashed')
            ax.quiver(*origin, *transformed_y, color='g', linestyle='dashed')
            ax.quiver(*origin, *transformed_z, color='b', linestyle='dashed')
        
            # Set plot limits and labels
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([-0, 1])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
    
        self.myCanvas.draw()

class MyThread(QThread):
    update_data = pyqtSignal()
 
    def __init__(self, parent=None):
        super(MyThread, self).__init__(parent)
        self.dataY1 = []
        self.dataY2 = []
 
    def run(self):
        while True:
            self.dataY1.append(np.random.random())
            self.dataY2.append(0.5)
            self.update_data.emit() # 发送更新信号
            time.sleep(1)# 线程暂停1秒
 
 
def run_plot():
    app = QApplication(sys.argv)
    m = Matplotib()
    m.show()
    sys.exit(app.exec_())

# thread = threading.Thread(target=run_plot)
# thread.start()


if __name__ == '__main__':
    thread = threading.Thread(target=run_plot)
    thread.start()
    while True:
        # print(1)
        pass