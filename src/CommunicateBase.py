import serial
import copy
import socket
import select
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


class TCP_Serverbase():
    def __init__(self):
        self.sok = None
        self.cons = []
        self.addrs = []

    def bind(self, addr, port):
        self.sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sok.bind((addr, port))

    def listen(self, num):
        self.sok.listen(num)

    def accept(self):
        con, addr = self.sok.accept()
        self.cons.append(con)
        self.addrs.append(addr)
        return self.cons.index(con)

    def tcp_send(self, indx, data):
        self.cons[indx].send(data)

    def tcp_read(self, index, number):
        return self.cons[index].recv(number)

    def tcp_close(self):
        self.sok.close()


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
        self.cmd_change_workmode = [
                                bytearray.fromhex('23 23 06 fe 01 01 00 00'),
                                bytearray.fromhex('23 23 06 fe 01 01 00 01')]
        self.cmd_add_bcc(self.cmd_change_workmode)
        self.cmd_change_A_range = [
                                bytearray.fromhex('23 23 0e fe 01 01 00 01'),
                                bytearray.fromhex('23 23 0e fe 01 01 00 02'),
                                bytearray.fromhex('23 23 0e fe 01 01 00 03')]
        self.cmd_add_bcc(self.cmd_change_A_range)
        self.cmdd_change_shake_limit = bytearray.fromhex('23 23 08 fe 01 0c 00')
        self.cmd_start_up = bytearray.fromhex('23 23 07 fe 01 01 00 01')
        self.cmd_stop_up = bytearray.fromhex('23 23 07 fe 01 01 00 00')
        self.cmd_tran_frequency = bytearray.fromhex('23 23 0d fe 01 00 00')
        self.cmd_work_mode = bytearray.fromhex('23 23 02 fe 01 00 00')
        self.cmd_shake_limit = bytearray.fromhex('23 23 04 fe 01 00 00')
        self.cmd_tempe = bytearray.fromhex('23 23 03 fe 01 00 00')
        self.cmd_A_range = bytearray.fromhex('23 23 0f fe 01 00 00')#A-Acceleration
        self.cmd_wireless_info = bytearray.fromhex('23 23 32 fe 01 00 00')
        self.cmd_add_bcc(self.cmd_start_up)
        self.cmd_add_bcc(self.cmd_stop_up)
        self.cmd_add_bcc(self.cmd_tran_frequency)
        self.cmd_add_bcc(self.cmd_work_mode)
        self.cmd_add_bcc(self.cmd_shake_limit)
        self.cmd_add_bcc(self.cmd_tempe)
        self.cmd_add_bcc(self.cmd_A_range)
        self.cmd_add_bcc(self.cmd_wireless_info)

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

    def cmd_add_data(self, cmd, data):
        cmd.append(data)


class TCP_Liten_Thread(QThread, CommProtocal):
    def __init__(self, sok, liten_keep, signals={}):
        QThread.__init__(self)
        CommProtocal.__init__(self)
        self.sok_server = sok
        self.signals = signals
        self.need_emit = 0
        self.daemon = True
        self.liten_keep = liten_keep
        self.position = 0

    def find_head(self, index):
        while self.liten_keep[0]:
            try:
                buf = self.sok_server.tcp_read(index, 1)
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

    def get_rsp_flag(self, index):
        # while self.sok_server.sok.in_waiting < 3:
        #     pass
        buf = self.sok_server.tcp_read(index, 3)
        return buf[0]

    def get_data_len(self, index):
        # while self.sok_server.sok.in_waiting < 2:
        #     pass
        buf = self.sok_server.tcp_read(index, 2)
        len = buf[0] + (buf[1] << 8)
        if len > 64:
            return -1
        return len

    def get_data(self, index,len):
        # while self.sok.in_waiting < len:
        #     pass
        buf = self.sok_server.tcp_read(index, len)
        return buf

    def get_check(self, index):
        buf = self.sok_server.tcp_read(index, 1)
        return buf
    
    def run(self):
        flag = 0
        while self.liten_keep[0]:
            readable, writable, errored = select.select([self.sok_server.sok], [], [], 0.1)
            for s in readable:
                if s == self.sok_server.sok:
                    self.sok_server.accept()
                    print('远程设备连接', str(self.sok_server.addrs[0]))
                    MSG.msg_info('远程设备连接', str(self.sok_server.addrs[0][0]), str(self.sok_server.addrs[0][1]))
                    flag = 1
                    break
            if flag:
                break
        while self.liten_keep[0]:
            # MSG.msg_info('find head')
            if self.find_head(0):
                print('find hand')
                rsp_flag = self.get_rsp_flag(0)
                if rsp_flag in self.signals.keys():
                    self.need_emit = self.signals[rsp_flag]
                len = self.get_data_len(0)
                if len == -1:
                    print('comm err')
                    continue
                self.data = self.get_data(0, len)
                self.get_check(0)
                if self.need_emit:
                    self.need_emit.emit(self.data)
                else:
                    print('no registor to emit')
            else:
                # print(self.liten_keep)
                break


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
    wireless_info = pyqtSignal(bytes)

    def __init__(self):
        QObject.__init__(self)
        CommProtocal.__init__(self)
        CommunicateBase.__init__(self)
        self.tcpserver = TCP_Serverbase()
        self.cmd_to_sign = {0x01: self.version_signal, 0x02: self.work_mode, 0x03: self.tempe, 0x04: self.shake_limit,
                             0x09: self.data_updata, 0x0d: self.fresh_speed, 0x0f: self.A_range, 0x32: self.wireless_info}
        self.liten_keep = [0]
        self.tcp_server_keep = [0]

    def tcp_server_start_listen(self):
        self.tcp_server_keep[0] = 1
        self.tcp_thread = TCP_Liten_Thread(self.tcpserver, self.tcp_server_keep, self.cmd_to_sign)
        self.tcp_thread.start()
        return 0

    def async_liten(self):
        self.liten_keep[0] = 1
        if self.serl:
            self.liten_thread = Liten_Thread(self.serl, self.liten_keep, self.cmd_to_sign)
            self.liten_thread.start()
            return 0
        else:
            print('connect first')
            return -1

    def tcp_stop_liten(self):
        self.tcp_server_keep[0] = 0

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

    def get_wireless_info(self):
        return self.write(bytes(self.cmd_wireless_info))

    def change_train_frequency(self, index):
        return self.write(bytes(self.cmd_change_trans[index]))

    def change_com_speed(self, index):
        return self.write(bytes(self.cmd_change_comspeeds[index]))

    def change_work_mode(self, index):
        return self.write(bytes(self.cmd_change_workmode[index]))

    def change_A_range(self, index):
        return self.write(bytes(self.cmd_change_A_range[index]))

    def change_shake_limit(self, data):
        cmdd = copy.deepcopy(self.cmdd_change_shake_limit)
        cmdd.extend(data)
        self.cmd_add_bcc(cmdd)
        return self.write(bytes(cmdd))

    def registor(self, cmd, func):
        if cmd in self.cmd_to_sign.keys():
            self.cmd_to_sign[cmd].connect(func)


motorcomm = MotorComm()
