import struct
import time
from collections import deque
import itertools
import numpy as np
import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from ctypes.wintypes import POINT
import ctypes.wintypes
import pyqtgraph.opengl as gl
import pyqtgraph as pg

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWinExtras import QtWin
from PyQt5.QtCore import QTimer, QThread
import win32api
import win32con
import win32gui
from Uis.Ui_MainWindow import Ui_MainWindow
from .Message import MSG
from .CommunicateBase import motorcomm


class GraphData():
    max_len = 1000
    x_data = deque([0 for i in range(max_len)], maxlen=max_len)
    y_data = deque([0 for i in range(max_len)], maxlen=max_len)
    z_data = deque([0 for i in range(max_len)], maxlen=max_len)
    # tempr_data = deque([1 for i in range(1000)], maxlen=1000)
    # curr_data = deque([1 for i in range(1000)], maxlen=1000)


class MainWindow(Ui_MainWindow):
    BorderWidth = 5

    def __init__(self, root):
        super(MainWindow, self).__init__()
        self.B_stop_up_flag = 0 #B-Button stop-0 up-1
        self.connect_flag = 0 #connect-1 disconnect-0
        self.show_len = 500
        self.info_time = 5000
        self.y = [-10,0,10]
        self.x = np.linspace(-20,20,100)
        self.screen_fresh_frequencys = [10, 20, 30, 40, 50, 60]
        self.timer = QTimer()
        self.timer_info = QTimer()
        self.graph_data = GraphData()
        self.setupUi(root)
        MSG.regist(self.textEdit)
        self.ini_graph()
        # self.mainwindow_area_config(root) #for window size change
        self.fresh_com()
        self.timer.timeout.connect(self.updata_graph)
        self.timer.start(1/30*1000) #设置计时间隔并启动
        self.timer_info.timeout.connect(self.get_info)
        self.show_3d_model()

    def init_solt(self):
        self.pushButton_upstop.clicked.connect(self.up_stop_solt)
        self.pushButton_connect.clicked.connect(self.connect_comm_solt)
        self.comboBox_com.clicked.connect(self.fresh_com)
        self.comboBox_tran_frequency.currentIndexChanged.connect(motorcomm.change_train_frequency)
        self.comboBox_fresh_frequency.currentIndexChanged.connect(self.screen_fresh_frequency_change)
        self.comboBox_bound.currentIndexChanged.connect(self.change_com_speed)
        motorcomm.registor(0x09, self.updata_graph_data)
        motorcomm.registor(0x0d, self.ui_tran_frequency_change)
        motorcomm.registor(0x02, self.work_mode_change)
        motorcomm.registor(0x04, self.shake_limit_change)
        motorcomm.registor(0x03, self.tempe_change)

    def show_3d_model(self):
        self.openGLWidget.setCameraPosition(distance=50, azimuth=-80)
        self.openGLWidget.show()
        # self.graphicsView.show()
        self.openGLWidget.setWindowTitle('pyqtgraph example: GLVolumeItem')

        # #b = gl.GLBoxItem()
        # #w.addItem(b)
        self.g = gl.GLGridItem()
        self.g.scale(2, 2, 1)
        # self.g.setSpacing(x=100, y=100, z=100, spacing=QVector3D(2, 2, 1000))
        self.openGLWidget.addItem(self.g)
        ax = gl.GLAxisItem()
        ax.setSize(x=25, y=25, z=25)
        self.openGLWidget.addItem(ax)
        yi = np.array([self.y[0]]*100)
        xz = np.array(list(itertools.islice(self.graph_data.x_data, len(self.graph_data.x_data) - 100, len(self.graph_data.x_data))))
        pts = np.vstack([self.x,yi,xz]).transpose()
        self.pltx = gl.GLLinePlotItem(pos=pts, width=2, antialias=True)
        self.openGLWidget.addItem(self.pltx)
        yi = np.array([self.y[1]]*100)
        yz = np.array(list(itertools.islice(self.graph_data.y_data, len(self.graph_data.y_data) - 100, len(self.graph_data.y_data))))
        pts = np.vstack([self.x,yi,yz]).transpose()
        self.plty = gl.GLLinePlotItem(pos=pts, width=2, antialias=True)
        self.openGLWidget.addItem(self.plty)
        yi = np.array([self.y[2]]*100)
        zz = np.array(list(itertools.islice(self.graph_data.z_data, len(self.graph_data.z_data) - 100, len(self.graph_data.z_data))))
        pts = np.vstack([self.x,yi,zz]).transpose()
        self.pltz = gl.GLLinePlotItem(pos=pts, width=2, antialias=True)
        self.openGLWidget.addItem(self.pltz)
        vector_a = np.array([[0, 0, 0], [self.graph_data.x_data[-1], self.graph_data.y_data[-1], self.graph_data.z_data[-1]]])
        self.plta = gl.GLLinePlotItem(pos=vector_a, color=pg.glColor((50, 50*1.3)), width=10, antialias=True)
        self.openGLWidget.addItem(self.plta)

    def mainwindow_area_config(self, root):
        # 主屏幕的可用大小（去掉任务栏）
        self._rect = QApplication.instance().desktop().availableGeometry(root)
        # self.resize(800, 600)
        root.setWindowFlags(Qt.Window
                            | Qt.FramelessWindowHint
                            | Qt.WindowSystemMenuHint
                            | Qt.WindowMinimizeButtonHint
                            | Qt.WindowMaximizeButtonHint
                            | Qt.WindowCloseButtonHint)
        # 增加薄边框
        style = win32gui.GetWindowLong(int(root.winId()), win32con.GWL_STYLE)
        win32gui.SetWindowLong(
            int(root.winId()), win32con.GWL_STYLE, style | win32con.WS_SIZEBOX)

        if QtWin.isCompositionEnabled():
            # 加上 Aero 边框阴影
            QtWin.extendFrameIntoClientArea(root, 0, 0, 0, 0)
        else:
            QtWin.resetExtendedFrame(root)

    def ini_graph(self):
        # self.graphicsView_x.enableAutoRange(False)
        self.graphicsView_x.setYRange(max=2, min=-2)
        self.graphicsView_y.setYRange(max=2, min=-2)
        self.graphicsView_z.setYRange(max=2, min=-2)
        self.graphicsView_x_2.setYRange(max=200, min=0)
        self.graphicsView_y_2.setYRange(max=200, min=0)
        self.graphicsView_z_2.setYRange(max=200, min=0)
        self.xp = self.graphicsView_x.plot(pen=(255, 0, 0))
        self.yp = self.graphicsView_y.plot(pen=(0, 255, 0))
        self.zp = self.graphicsView_z.plot(pen=(0, 0, 255))
        self.xp2 = self.graphicsView_x_2.plot(pen=(255, 0, 0))
        self.yp2 = self.graphicsView_y_2.plot(pen=(0, 255, 0))
        self.zp2 = self.graphicsView_z_2.plot(pen=(0, 0, 255))
        # self.temprp = self.graphicsView_tempr.plot(pen=(255, 0, 255))
        # self.currp = self.graphicsView_curr.plot(pen=(255, 255, 0))

    def work_mode_change(self, data):
        if len(data) != 1:
            print('work_mode_change len err')
            return
        self.comboBox_work_mode.setCurrentIndex(struct.unpack('<b',data)[0])

    def shake_limit_change(self, data):
        if len(data) != 12:
            print('shake_limit_change len err')
            return
        self.lineEdit_shakelimit_x.setText('%.2f'%struct.unpack('<f', data[0:4])[0])
        self.lineEdit_shakelimit_y.setText('%.2f'%struct.unpack('<f', data[4:8])[0])
        self.lineEdit_shakelimit_z.setText('%.2f'%struct.unpack('<f', data[8:12])[0])

    def tempe_change(self, data):
        if len(data) != 4:
            print('tempe_change len err')
            return
        self.label_tempe.setText('%2d'%struct.unpack('<f', data)[0])

    def screen_fresh_frequency_change(self, index):
        self.timer.stop()
        self.timer.start(1/self.screen_fresh_frequencys[index]*1000)

    def ui_tran_frequency_change(self, data):
        if len(data) != 1:
            print('ui_tran_frequency_change len err')
            return
        self.data_to_comb_indexs = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a]
        self.comboBox_tran_frequency.setCurrentIndex(self.data_to_comb_indexs.index(data[0]))

    def A_range_change(self, data):
        if len(data) != 1:
            print('A_range_change len err')
            return
        self.comboBox_A_range.setCurrentText(data[0]+1)

    def get_info(self):
        print('get info')
        motorcomm.get_tran_frequency()
        QThread.msleep(50)
        motorcomm.get_work_mode()
        QThread.msleep(50)
        motorcomm.get_tempe()
        QThread.msleep(50)
        motorcomm.get_shake_limit()
        QThread.msleep(50)
        motorcomm.get_A_range()

    def change_com_speed(self, index):
        if not self.connect_flag:
            return
        motorcomm.change_com_speed(index)
        com = self.comboBox_com.currentText()
        bound = int(self.comboBox_bound.currentText())
        motorcomm.close()
        self.connect_flag = 0
        if motorcomm.connect(com, bound) != -1:
            motorcomm.async_liten()
            self.connect_flag = 1
            MSG.msg_info('重新连接', str(bound))
        else:
            MSG.msg_info('重新连接失败', str(bound))
            self.pushButton_connect.setText('连接')

    def fresh_com(self):
        temp_coms = motorcomm.getall_com()
        self.comboBox_com.clear()
        for com in temp_coms:
            self.comboBox_com.addItem(com.name)

    def up_stop_solt(self):
        if not self.B_stop_up_flag:
            MSG.msg_info('start data up!')
            if motorcomm.start_data_up() != -1:
                if self.timer_info.isActive():
                    self.timer_info.stop()
                self.pushButton_upstop.setText('Stop')
                self.B_stop_up_flag = 1
        else:
            MSG.msg_info('stop data up!')
            if motorcomm.stop_data_up() != -1:
                if not self.timer_info.isActive():
                    self.timer_info.start(self.info_time)
                self.pushButton_upstop.setText('Start')
                self.B_stop_up_flag = 0

    def connect_comm_solt(self):
        com = self.comboBox_com.currentText()
        bound = int(self.comboBox_bound.currentText())
        if not self.connect_flag:
            if motorcomm.connect(com, bound) != -1:
                motorcomm.async_liten()
                self.timer_info.start(self.info_time)
                self.connect_flag = 1
                self.pushButton_connect.setText('断开')
                self.get_info()
        else:
            self.timer_info.stop()
            motorcomm.stop_liten()
            motorcomm.close()
            self.connect_flag = 0
            self.pushButton_connect.setText('连接')

    def updata_graph_data(self, data):
        if len(data) != 12:
            print('updata_graph_data len err')
            return
        self.graph_data.x_data.append(struct.unpack('<f', data[0:4])[0])
        self.graph_data.y_data.append(struct.unpack('<f', data[4:8])[0])
        self.graph_data.z_data.append(struct.unpack('<f', data[8:12])[0])

    def updata_graph(self):
        x_count_data = list(itertools.islice(self.graph_data.x_data, len(self.graph_data.x_data) - self.show_len, len(self.graph_data.x_data)))
        y_count_data = list(itertools.islice(self.graph_data.y_data, len(self.graph_data.y_data) - self.show_len, len(self.graph_data.y_data)))
        z_count_data = list(itertools.islice(self.graph_data.z_data, len(self.graph_data.z_data) - self.show_len, len(self.graph_data.z_data)))
        self.xp.setData(self.graph_data.x_data)
        self.yp.setData(self.graph_data.y_data)
        self.zp.setData(self.graph_data.z_data)

        xf = np.fft.rfft(x_count_data)
        xfp = np.abs(xf)[1:]
        self.xp2.setData(xfp)
        yf = np.fft.rfft(y_count_data)
        yfp = np.abs(yf)[1:]
        self.yp2.setData(yfp)
        zf = np.fft.rfft(z_count_data)
        zfp = np.abs(zf)[1:]
        self.zp2.setData(zfp)
        self.openGLWidget.removeItem(self.plta)
        self.openGLWidget.removeItem(self.pltx)
        self.openGLWidget.removeItem(self.plty)
        self.openGLWidget.removeItem(self.pltz)
        yi = np.array([self.y[0]]*100)
        xz = np.array((xfp[:100])/10)
        pts = np.vstack([self.x,yi,xz[:100]]).transpose()
        self.pltx = gl.GLLinePlotItem(pos=pts, color=pg.glColor((1, 50*1.3)), width=1, antialias=True)
        self.openGLWidget.addItem(self.pltx)
        yi = np.array([self.y[1]]*100)
        yz = np.array((yfp[:100])/10)
        pts = np.vstack([self.x,yi,yz[:100]]).transpose()
        self.plty = gl.GLLinePlotItem(pos=pts, color=pg.glColor((25, 50*1.3)), width=2, antialias=True)
        self.openGLWidget.addItem(self.plty)
        yi = np.array([self.y[2]]*100)
        zz = np.array((zfp[:100])/10)
        pts = np.vstack([self.x,yi,zz[:100]]).transpose()
        self.pltz = gl.GLLinePlotItem(pos=pts, color=pg.glColor((40, 50*1.3)), width=2, antialias=True)
        self.openGLWidget.addItem(self.pltz)
        vector_a = np.array([[0, 0, 0], [self.graph_data.x_data[-1], self.graph_data.y_data[-1], self.graph_data.z_data[-1]]])*10
        self.plta = gl.GLLinePlotItem(pos=vector_a, color=pg.glColor((50, 50*1.3)), width=5, antialias=True)
        self.openGLWidget.addItem(self.plta)
        # z = np.array(list(itertools.islice(self.graph_data.x_data, len(self.graph_data.x_data) - 100, len(self.graph_data.x_data))))
        # pts = np.vstack([self.x,yi,z]).transpose()
        # plt = gl.GLLinePlotItem(pos=pts, width=2, antialias=True)
        # self.openGLWidget.addItem(plt)
        # self.graphicsView_x.enableAutoRange('xy', False)

    def nativeEvent(self, eventType, message):
        retval, result = super(Window, self).nativeEvent(eventType, message)
        if eventType == "windows_generic_MSG":
            msg = ctypes.wintypes.MSG.from_address(message.__int__())
            if msg.message == win32con.WM_NCCALCSIZE:
                # 拦截不显示顶部的系统自带的边框
                return True, 0
            if msg.message == win32con.WM_GETMINMAXINFO:
                # 当窗口位置改变或者大小改变时会触发该消息
                info = ctypes.cast(
                    msg.lParam, ctypes.POINTER(MINMAXINFO)).contents
                # 修改最大化的窗口大小为主屏幕的可用大小
                info.ptMaxSize.x = self._rect.width()
                info.ptMaxSize.y = self._rect.height()
                # 修改放置点的x,y坐标为0,0
                info.ptMaxPosition.x, info.ptMaxPosition.y = 0, 0
            if msg.message == win32con.WM_NCHITTEST:
                # 获取鼠标移动经过时的坐标
                x = win32api.LOWORD(msg.lParam) - self.frameGeometry().x()
                y = win32api.HIWORD(msg.lParam) - self.frameGeometry().y()
                w, h = self.width(), self.height()
                lx = x < self.BorderWidth
                rx = x > w - self.BorderWidth
                ty = y < self.BorderWidth
                by = y > h - self.BorderWidth
                # 左上角
                if (lx and ty):
                    return True, win32con.HTTOPLEFT
                # 右下角
                if (rx and by):
                    return True, win32con.HTBOTTOMRIGHT
                # 右上角
                if (rx and ty):
                    return True, win32con.HTTOPRIGHT
                # 左下角
                if (lx and by):
                    return True, win32con.HTBOTTOMLEFT
                # 上
                if ty:
                    return True, win32con.HTTOP
                # 下
                if by:
                    return True, win32con.HTBOTTOM
                # 左
                if lx:
                    return True, win32con.HTLEFT
                # 右
                if rx:
                    return True, win32con.HTRIGHT
                # 标题
                return True, win32con.HTCAPTION
        return retval, result