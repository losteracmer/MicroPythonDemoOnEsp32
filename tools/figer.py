import machine
import time
import utime

"""
HiLink 海凌科 的指纹模组，适用于 ZW101、ZW0906 （理论上协议一致都可以)
"""

code_ok = "Success"
code_err_pkg = "Invalid package"
code_not_match = "Figer not match"

search_failed = "Search failed"
gen_char_failed = "Gen char failed"
invalid_package = "Invalid package"
no_figer_on_it = "No figer on it"
save_image_failed = "Save image failed"
error_code = {
    b'\x00': code_ok,
    b'\x01': invalid_package,
    b'\x02': no_figer_on_it,
    b'\x03': save_image_failed,
    b'\x04': "Image too dry",
    b'\x05': "Image too damp",
    b'\x06': "Invalid checksum",
    b'\x07': "Invalid package",
    b'\x08': code_not_match,
    b'\x09': search_failed,
    b'\x0A': gen_char_failed,
    b'\x0B': "Invalid device",
    b'\x0C': "Invalid device",
    b'\x0D': "Invalid device",
    b'\x0E': "Invalid device",
    b'\x0F': "Invalid device",
    b'\x10': "Invalid device",
    b'\x11': "Invalid device",
    b'\x12': "Invalid device",
    b'\x13': "Invalid device",
    b'\x14': "Invalid device",
    b'\x15': "Invalid device",
    b'\x16': "Invalid device",
    b'\x17': "Invalid device",
    b'\x18': "Invalid device",
    b'\x19': "Invalid device",
    b'\x1A': "Invalid device",
    b'\x1B': "Invalid device",
    b'\x1C': "Invalid device",
    b'\x1D': "Invalid device",
    b'\x1E': "Invalid device",
    b'\x1F': "Invalid device",
    b'\x20': "Invalid device",
    b'\x21': "Invalid device",
    b'\x22': "Invalid device",
    b'\x23': "Invalid device",
    b'\x24': "Invalid device",
    b'\x25': "Invalid device",
    b'\x26': "Invalid device",
    b'\x27': "Invalid device",
    b'\x28': "Invalid device",
    b'\x29': "Invalid device",
    b'\x2A': "Invalid device",
    b'\x2B': "Invalid device",
    b'\x2C': "Invalid device",
    b'\x2D': "Invalid device",
    b'\x2E': "Invalid device",
    b'\x2F': "Invalid device",
    b'\x30': "Invalid device",
    b'\x31': "Invalid device",
    b'\x32': "Invalid device",
    b'\x33': "Invalid device",
    b'\x34': "Invalid device",
    b'\x35': "Invalid device",
    b'\x36': "Invalid device",
    b'\x37': "Invalid device",
    b'\x38': "Invalid device",
    b'\x39': "Invalid device",
    b'\x3A': "Invalid device",
    b'\x3B': "Invalid device",
    b'\x3C': "Invalid device",
    b'\x3D': "Invalid device",
    b'\x3E': "Invalid device",
    b'\x3F': "Invalid device",
    b'\x40': "Invalid device",
}

command_ok = "OK"


def calculate_checksum(data):
    # print("calculate checksum ", data.hex())
    return sum(data) % 65535  # 超过两个字节的忽略


class BaseProtocol:
    byteorder = 'big'

    def __init__(self, port, tx, rx, baudrate=57600):
        # print("init,{}, {}, {}, {} ".format(port, tx, rx, baudrate))
        # timeout=2000, timeout_char=1000
        self.uart = machine.UART(port, baudrate=baudrate, tx=tx, rx=rx, )
        # print("first", self.uart)

    # 发送数据给下位机
    def send_to_lower_machine(self, data):
        print(">> send:", data.hex())
        # print(self.uart)
        self.uart.write(data)

    # 从下位机接收数据
    def receive_from_lower_machine(self, timeout_ms=5000):
        # if self.uart.any():
        #     return self.uart.read()
        start_time = utime.ticks_ms()  # 记录开始时间

        # 读取数据
        data = b''
        step = 'first'  # begin, first_package, get_length, other
        while utime.ticks_diff(utime.ticks_ms(), start_time) < timeout_ms:
            # print("into while")
            if not self.uart.any():
                continue
            if step == 'first':
                # 如果串口有数据可读
                data += self.uart.read(1)  # 读取2个字节
                # print("data f get", data)
                if data == b'\xEF':
                    step = 'first_package'
                else:
                    step = 'other'
                    continue
            elif step == 'first_package':
                data += self.uart.read(1 + 4 + 1 + 2)  # 1:\x01 4:地址，1:包表示，2:包长
                package_len = int.from_bytes(data[-2:], self.byteorder)
                data += self.uart.read(package_len)
                # print("data p get", data)
                return data
            elif step == 'other':
                data += self.uart.read()  # 读完缓冲区即可。
                # print("data o get", data)
                return data
            utime.sleep_ms(10)  # 暂停一段时间，避免过于频繁地读取串口数据

    def receive_command_from_lower_machine(self, time_out=5000):
        data = self.receive_from_lower_machine(timeout_ms=time_out)
        # print("Receive data", data.hex())
        if data:
            try:
                package_identifier, payload = self.parse_received_packet(data)
                if package_identifier in error_code.keys():
                    return error_code[package_identifier], payload
                else:
                    return "Unknown package", None
            except Exception as e:
                print("parse packet error:", e, data)
                return None, None
        return None, None

    def send_command_to_lower_machine(self, device_address, command, params):
        header = b"\xEF\x01"
        package_identifier = b"\x01"
        if params:
            package_length = len(params) + 3  # 2是校验和+指令
        else:
            package_length = 3
        data = package_identifier + \
               package_length.to_bytes(2, self.byteorder) + \
               command + params
        checksum = sum(data) % 65535
        packet1 = header + device_address + package_identifier + package_length.to_bytes(2, self.byteorder) + \
                  command + params + checksum.to_bytes(2, self.byteorder)
        packet = packet1
        self.send_to_lower_machine(packet)

    def pack_command_packet(self, device_address, command, params):
        header = b"\xEF\x01"
        package_identifier = b"\x01"
        if params:
            package_length = len(params) + 3  # 2是校验和+指令
        else:
            package_length = 3
        data = package_identifier + \
               package_length.to_bytes(2, self.byteorder) + \
               command + params
        checksum = sum(data) % 65535
        packet = header + device_address + package_identifier + package_length.to_bytes(2, self.byteorder) + \
                 command + params + checksum.to_bytes(2, self.byteorder)
        return packet

    # 解析收到的数据包
    def parse_received_packet(self, data):
        print("<< received: ", data.hex())
        if len(data) == 0:
            raise Exception("Null data")
        if len(data) < 9:
            raise Exception("illegal package")
        header = data[:2]
        device_address = data[2:6]
        package_identifier = data[6]
        package_length = data[7:9]
        # 将超长的数据截断，防止溢出
        length_int = int.from_bytes(package_length, self.byteorder)
        if length_int < len(data[9:]):
            out_of_data = data[9 + length_int:]
            ack_code, payload = self.parse_received_packet(out_of_data)
            print("!!!! out of data ack_code:{}, payload:{}", ack_code, payload)
            data = data[:length_int]
        ack_code = data[9:10]
        payload = data[10:-2]
        checksum = data[-2:]
        data1 = data[6:-2]
        calculated_checksum = sum(data1) % 65535
        print("[] Parse Rec: pkg_id:{}, pkb_len:{}, ack_code:{}, payload:{}, checksum:{}, [{}]".format(
            package_identifier, package_length, ack_code, payload, checksum, calculated_checksum))
        if calculated_checksum.to_bytes(2, self.byteorder) != checksum:
            raise Exception("Checksum error")
        return ack_code, payload

    def show_received_message(self, data):
        print("show received message:", data)


# Pin 1-6: v_sensor touch_out VCC tx rx GND
class ZW101(BaseProtocol):
    # args: rx -> figer-module.tx tx -> figer-module.rx
    def __init__(self, port, tx, rx, baudrate=57600, vcc=0, sensor=0):
        super().__init__(port, tx, rx, baudrate)
        self.device_address = b'\xff\xff\xff\xff'
        self.power_pin = machine.Pin(vcc, machine.Pin.OUT)
        self.sensor_pin = machine.Pin(sensor, machine.Pin.IN)
        self.sensor_gate = False
        self.sensor_handler = None
        self.power_time = None

    def close_gate(self):
        # print("gate close")
        self.sensor_gate = False

    def open_gate(self):
        # print("gate open")
        self.sensor_gate = True

    def sensor_handle(self, pin):
        # if event - power_time < 100ms, skip it
        print("tiggered: ", pin.value())
        # todo 开一个触摸事件队列，如果连续触发4次，则激活录入模式
        #
        if self.power_time and utime.ticks_diff(utime.ticks_ms(), self.power_time) < 1000:
            self.control_ibln(color='white')
            utime.sleep_ms(100)
            self.control_ibln(color='none')
            return

        # print("======= sensor be triggerd", pin.value(), self.sensor_gate)
        if not self.sensor_handler:
            return
        if not self.sensor_gate:
            return
        if not pin.value():
            return
        if not self.sensor_handler:
            return
        self.close_gate()
        self.sensor_handler(self)

    def before_low_energy(self):
        # register wake up trigger use sensor
        print("before_low_energy")
        # self.sensor_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=None)
        self.sensor_pin.irq(trigger=machine.Pin.WAKE_HIGH, wake=machine.SLEEP)
        print("pin value: ", self.sensor_pin.value())
        # machine.lightsleep()

    def after_low_energy(self):
        self.sensor_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=self.sensor_handle)

    def register_sensor_handler(self, handler):
        # skip sleep event(-1-0) when close power
        self.sensor_handler = handler
        self.sensor_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=self.sensor_handle)

    def send_command(self, command, params):
        self.send_command_to_lower_machine(self.device_address, command, params)

    def send_and_receive(self, command, params, timeout=5000):
        self.send_command_to_lower_machine(self.device_address, command, params)
        # time.sleep(timeout)
        status, received_data = self.receive_command_from_lower_machine(time_out=timeout)
        # self.show_received_message(received_data)
        return status, received_data

    def power_on(self):
        self.power_pin.value(1)
        self.power_time = utime.ticks_ms()
        utime.sleep_ms(100)
        receive_power_on = self.receive_from_lower_machine(timeout_ms=1000)
        if receive_power_on == b'0xff55':
            print("power on:", receive_power_on)
        else:
            print("power on error:", receive_power_on)

    def power_off(self):
        self.power_pin.value(0)
        # self.sensor_gate = True

    def control_ibln(self, color='blue'):
        color_map = {
            'blue': b'\x01',
            'green': b'\x02',
            'red': b'\x04',
            'white': b'\x07',
            'none': b'\x00',
        }
        color_bit = b'\x01'
        if color in color_map.keys():
            color_bit = color_map.get(color)
        # breathe color
        cmd, payload = b'\x3c', b'\x01' + color_bit + b'\x00\x01'
        print("=> PS_ControlIBLN:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_ControlIBLN:", receive)

    def sleep(self):
        cmd, payload = b'\x33', b''
        print("=> PS_Sleep:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        self.open_gate()
        print("<= PS_Sleep:", receive)

    def get_image(self):
        cmd, payload = b'\x01', b''
        print("=> PS_GetImage:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_GetImage:", receive)
        return ack, receive

    def gen_char(self):
        cmd, payload = b'\x02', b'\x01'  # bufferID
        print("=> PS_GenChar:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_GenChar:", receive)
        return ack, receive

    def match(self):
        cmd, payload = b'\x03', b''
        print("=> PS_Match:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_Match:", receive)

    # search 搜索指纹，返回确认码 和 得分
    def search(self):
        # ZW101 有个bug，如果识别失败，则会返回FF55
        cmd, payload = b'\x04', b'\x01\x00\x00\x00\x32'  # buffer_id, start_page, page_num
        print("=> PS_Search:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_Search:", receive)
        if ack == code_ok:
            fiter_id, score = receive[:2], receive[2:]
            print("== search ok,id:{} score:{}".format(int.from_bytes(fiter_id, self.byteorder),
                                                       int.from_bytes(score, self.byteorder)))
            return ack, receive
        print("== search failed")
        return code_not_match, receive

    def reg_model(self):
        cmd, payload = b'\x05', b''
        print("=> PS_RegModel:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_RegModel:", receive)

    def store_char(self):
        cmd, payload = b'\x06', b''
        print("=> PS_StoreChar:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_StoreChar:", receive)

    def auto_identify(self):
        cmd, payload = b'\x32', b'\x00\xff\xff\x00\x00'  # 1:score level?, 2:id, 2:arg?
        print("=> PS_AutoIdentify:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_AutoIdentify:", receive)
        args1, id_code, score = receive[:1], receive[1:3], receive[3:5]
        print("== args:{}, id:{}, score:{}".format(args1, int.from_bytes(id_code, self.byteorder),
                                                   int.from_bytes(score, self.byteorder)))

    def read_sys_para(self):
        cmd, payload = b'\x0f', b''
        print("=> PS_ReadSysPara:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_ReadSysPara:", receive)
        if not receive:
            return
        register_cnt, figer_size, colume_size, score_level, address, package_size, baudrate = \
            receive[:2], receive[2:4], receive[4:6], receive[6:8], receive[8:12], receive[12:14], receive[14:]
        print(
            "== register_cnt:{}, figer_size:{}, colume_size:{}, score_level:{}, address:{}, package_size:{}, baudrate:{}".
            format(register_cnt, figer_size, colume_size, score_level, address, package_size, baudrate))

    def read_index_table(self):
        cmd, payload = b'\x1f', b'\x00'
        print("=> PS_ReadIndexTable:", cmd.hex(), payload.hex())
        ack, receive = self.send_and_receive(cmd, payload)
        print("<= PS_ReadIndexTable:", receive)
        # print(format(receive, '02B'))
        print('--------')
        for byte_index, byte_value in enumerate(receive):
            for bit_index in range(8):
                if byte_value & (1 << bit_index):
                    print("| {}\t |".format(byte_index * 8 + bit_index))
        print('--------')

    # auto_enroll 自动注册指纹指令，figer_id 指纹id
    def auto_enroll(self, figer_id, input_cnt=5):
        # 将figer_id转化成byte
        figer_id_byte = figer_id.to_bytes(2, self.byteorder)
        input_cnt_byte = input_cnt.to_bytes(1, self.byteorder)
        cmd, payload = b'\x31', figer_id_byte + input_cnt_byte + b'\x00\x00'  # 2:id 1:input_cnt, 2:arg?
        print("=> PS_AutoIdentify:", cmd.hex(), payload.hex())
        self.send_command(cmd, payload)
        while True:
            ack, receive = self.receive_command_from_lower_machine()
            if not receive:
                continue
            arg1, arg2 = receive[:1], receive[1:2]
            if ack == code_ok:  # 成功
                if arg1 == b'\x00':  # 合法性检测
                    print("== this is {} register".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x01':  # 获取图像
                    print("== get image success {}".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x02':  # 生产特征
                    print("== gen char success {}".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x03':  # 手指离开
                    print("== please move your figer from module {}".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x04':  # 合成木板
                    print("== gen tmp success {}".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x05':  # 注册校验
                    print("== register checking... {}".format(int.from_bytes(arg2, self.byteorder)))
                elif arg1 == b'\x06':  # 存储模板
                    print("== save tmp")
                    break
            elif ack == invalid_package:
                print("== register failed !")
                break
            else:
                print("== register failed, ack", ack)

    def auto_search(self):
        ack, rec = self.get_image()
        if ack != code_ok:
            return ack, rec
        ack, rec = self.gen_char()
        if ack != code_ok:
            return ack, rec
        return self.search()

