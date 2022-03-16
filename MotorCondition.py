import sys
from PyQt5.QtWidgets import QApplication
sys.path.append('./Uis')    #自动生成的ui文件控件提升后无法准确识别文件路径这里加上
sys.path.append('./src')

from src.Root import Root

if __name__ == '__main__':

    app = QApplication(sys.argv)
    root = Root()
    # root.time_to_updata_graph()
    root.show()
    sys.exit(app.exec_())
