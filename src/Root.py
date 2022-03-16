from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QWidget, QVBoxLayout
# from PyQt5.QtGui import *

from src.MainWindow import MainWindow
from .Message import MSG
from Uis.MyDevice import TitleBar

RES_PATH = './res/'

#MainWindow
WINDOW_QSS = RES_PATH + "theme.qss"
WINDOW_ICON = RES_PATH + "win.png"
def readQss(style):
    with open(style, 'r') as f:
        return f.read()


class Root():
    Margins = 5
    def __init__(self):
        global message
        self.rootwindow = QMainWindow()
        self.rootwindow.setWindowFlags(Qt.FramelessWindowHint)

        self.mainwindow = MainWindow(self.rootwindow)
        # # 预留边界用于实现无边框窗口调整大小
        self.mainwindow.gridLayout_3.setContentsMargins(
            self.Margins, self.Margins, self.Margins, self.Margins)
        self.titleBar = TitleBar(self.rootwindow)
        self.titleBar.SetIcon(QPixmap(WINDOW_ICON))
        self.mainwindow.gridLayout_16.addWidget(self.titleBar)
        self.mainwindow.gridLayout_16.setSpacing(0)
        self.mainwindow.gridLayout_16.addWidget(self.mainwindow.groupBox_2)
        self.mainwindow.gridLayout_16.setContentsMargins(
             self.Margins, self.Margins, self.Margins, self.Margins)
        self.rootwindow.showNormal()
        styleFile = './Uis/style.qss'
        qssStyle = readQss(styleFile)
        self.rootwindow.setStyleSheet(qssStyle)
        self.mainwindow.init_solt()

    def show(self):
        self.rootwindow.show()

