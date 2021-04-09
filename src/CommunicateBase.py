import serial
import serial.tools.list_ports

from PyQt5.QtCore import pyqtSignal, QObject, QThread
from .Message import MSG


# # 单例模式修饰器
# def singleton(cls):
#     _instance = {}

#     def _singleton(*args, **kwargs):
#         # 先判断这个类有没有对象
#         if cls not in _instance:
#             _instance[cls] = cls(*args, **kwargs)  # 创建一个对象,并保存到字典当中
#         # 将实例对象返回
#         return _instance[cls]

#     return _singleton


# @singleton
class CommunicateBase():
    def __init__(self):
        self.serl = None

    def getall_com(self):
        return serial.tools.list_ports.comports()

    def connect(self, com='', freq=115200, timeout=0.5):
        try:
            MSG.msg_info('connect', com)
            self.serl = serial.Serial(com, freq, timeout=timeout)
            return 0
        except Exception as e:
            MSG.msg_info(str(e))
            return -1

    def write(self, data):
        try:
            return self.serl.write(data)
        except Exception:
            MSG.msg_info('串口发送错误')
            return -1

    def read(self, num):
        try:
            return self.serl.read(num)
        except Exception:
            MSG.msg_info('串口读取错误')
            return -1

    def close(self):
        self.serl.close()
        MSG.msg_info('断开连接')

    def __del__(self):
        if not self.serl:
            self.serl.close()


class CommProtocal():
    def __init__(self):
        self.head = bytearray('##', encoding='ascii')
        self.cmd = 0
        self.response = 0
        self.len = 0
        self.data = bytearray('', encoding='ascii')
        self.cmd_change_trans = [
                                bytearray.fromhex('23 23 0c fe 01 01 00 00'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 01'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 02'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 03'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 04'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 05'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 06'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 07'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 08'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 09'),
                                bytearray.fromhex('23 23 0c fe 01 01 00 0a')]
        self.cmd_add_bcc(self.cmd_change_trans)
        self.cmd_change_comspeeds = [
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 00'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 01'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 02'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 03'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 04'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 05'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 06'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 07'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 08'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 09'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 0a'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 0b'),
                                bytearray.fromhex('23 23 0b fe 01 05 00 08 01 00 00 0c')]
        self.cmd_add_bcc(self.cmd_change_comspeeds)
        self.cmd_start_up = bytearray.fromhex('23 23 07 fe 01 01 00 01')
        self.cmd_add_bcc(self.cmd_start_up)
        self.cmd_stop_up = bytearray.fromhex('23 23 07 fe 01 01 00 00')
        self.cmd_add_bcc(self.cmd_stop_up)
        self.cmd_tran_frequency = bytearray.fromhex('23 23 0d fe 01 00 00')
        self.cmd_add_bcc(self.cmd_tran_frequency)
        self.cmd_work_mode = bytearray.fromhex('23 23 02 fe 01 00 00')
        self.cmd_add_bcc(self.cmd_work_mode)
        self.cmd_shake_limit = bytearray.fromhex('23 23 04 fe 01 00 00')
        self.cmd_add_bcc(self.cmd_shake_limit)
        self.cmd_tempe = bytearray.fromhex('23 23 03 fe 01 00 00')
        self.cmd_add_bcc(self.cmd_tempe)
        self.cmd_A_range = bytearray.fromhex('23 23 0f fe 01 00 00')#A-Acceleration
        self.cmd_add_bcc(self.cmd_A_range)

    def cmd_add_bcc(self, cmds):
        if isinstance(cmds[0], bytearray):
            for cmd in cmds:
                bcc = cmd[0]
                for b in cmd[1:]:
                    bcc ^= b
                cmd.append(bcc)
        else:
            bcc = cmds[0]
            for b in cmds[1:]:
                bcc ^= b
            cmds.append(bcc)


class Liten_Thread(QThread, CommProtocal):
    def __init__(self, serl, liten_keep, signals={}):
        QThread.__init__(self)
        CommProtocal.__init__(self)
        self.serl = serl
        self.signals = signals
        self.need_emit = 0
        self.daemon = True
        self.liten_keep = liten_keep
        self.position = 0

    def find_head(self):
        while self.liten_keep[0]:
            try:
                buf = self.serl.read(1)
                # print(buf)
                if buf:
                    if buf[0] == self.head[self.position]:
                        # print(buf)
                        self.position += 1
                    else:
                        self.position = 0
                    if self.position == 2:
                        self.position = 0
                        return 1
            except Exception as e:
                print(e)
                return 0

    def get_rsp_flag(self):
        while self.serl.in_waiting < 3:
            pass
        buf = self.serl.read(3)
        return buf[0]

    def get_data_len(self):
        while self.serl.in_waiting < 2:
            pass
        buf = self.serl.read(2)
        len = buf[0] + (buf[1] << 8)
        if len > 64:
            return -1
        return len

    def get_data(self, len):
        while self.serl.in_waiting < len:
            pass
        buf = self.serl.read(len)
        return buf

    def get_check(self):
        buf = self.serl.read(1)
        return buf

    def run(self):
        while self.liten_keep[0]:
            # MSG.msg_info('find head')
            if self.find_head():
                rsp_flag = self.get_rsp_flag()
                if rsp_flag in self.signals.keys():
                    self.need_emit = self.signals[rsp_flag]
                len = self.get_data_len()
                if len == -1:
                    print('comm err')
                    continue
                self.data = self.get_data(len)
                self.get_check()
                if self.need_emit:
                    self.need_emit.emit(self.data)
                else:
                    print('no registor to emit')
            else:
                # print(self.liten_keep)
                break


class MotorComm(QObject, CommunicateBase, CommProtocal):
    version_signal = pyqtSignal(bytes)
    data_updata = pyqtSignal(bytes)
    fresh_speed = pyqtSignal(bytes)
    tempe = pyqtSignal(bytes)
    shake_limit = pyqtSignal(bytes)
    work_mode = pyqtSignal(bytes)
    A_range = pyqtSignal(bytes)

    def __init__(self):
        QObject.__init__(self)
        CommProtocal.__init__(self)
        CommunicateBase.__init__(self)
        self.cmd_to_sign = {0x01: self.version_signal, 0x02: self.work_mode, 0x03: self.tempe, 0x04: self.shake_limit,
                             0x09: self.data_updata, 0x0d: self.fresh_speed, 0x0f: self.A_range}
        self.liten_keep = [0]

    def async_liten(self):
        self.liten_keep[0] = 1
        if self.serl:
            self.liten_thread = Liten_Thread(self.serl, self.liten_keep, self.cmd_to_sign)
            self.liten_thread.start()
            return 0
        else:
            print('connect first')
            return -1

    def stop_liten(self):
        self.liten_keep[0] = 0

    def start_data_up(self):
        return self.write(bytes(self.cmd_start_up))

    def stop_data_up(self):
        return self.write(bytes(self.cmd_stop_up))

    def get_work_mode(self):
        return self.write(bytes(self.cmd_work_mode))

    def get_tempe(self):
        return self.write(bytes(self.cmd_tempe))

    def get_shake_limit(self):
        return self.write(bytes(self.cmd_shake_limit))

    def get_tran_frequency(self):
        return self.write(bytes(self.cmd_tran_frequency))

    def get_A_range(self):
        return self.write(bytes(self.cmd_A_range))

    def change_train_frequency(self, index):
        return self.write(bytes(self.cmd_change_trans[index]))

    def change_com_speed(self, index):
        return self.write(bytes(self.cmd_change_comspeeds[index]))

    def registor(self, cmd, func):
        if cmd in self.cmd_to_sign.keys():
            self.cmd_to_sign[cmd].connect(func)


motorcomm = MotorComm()
