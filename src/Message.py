class Message():
    def regist(self, out):
        self.msg_out = out

    def msg_info(self, *args):
        txt_string = ''
        for txt in args:
            txt_string += txt
            txt_string += ' '
        self.msg_out.append(txt_string)


MSG = Message()
