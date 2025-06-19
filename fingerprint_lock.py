print("welcome to micropython figer lock system")
from machine import Pin
from tools.servo import Servo
from tools.figer import *
import tools.low_energy as lem
from tools import pwmled as led
import network
import mha
import _thread as thread
thread.stack_size(16*1024)

# CONFIG
wake_time = 20
BROKER_ADDR = "192.168.31.135"
module_low_energy = False
use_ha = True
sensor = None

# 连接WiFi
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)  # 创建一个STA接口对象
    print(wlan)
    wlan.active(True)  # 激活STA接口

    # 连接WiFi
    wlan.connect(ssid, password)

    # 等待连接成功
    while not wlan.isconnected():
        print("Connecting to WiFi...")
        time.sleep(1)

    print("Connected to WiFi:", ssid)
    print("IP Address:", wlan.ifconfig()[0])

class ServoLock:
    def __init__(self, pin, ha_enable=False):
        self.servo = Servo(Pin(22), max_us=2500, angle=180)
        self.servo.write_angle(0)
        if ha_enable:
            # check WI-FI is connected
            connect_wifi("Avengers", "qwertYUIOP")
            device = mha.HADevice("001122AABBC0")  # (binascii.hexlify(machine.unique_id()).decode('utf-8'))
            mqtt = mha.HAMqtt(device)
            self.mqtt = mqtt

            device.set_name("My Fingerprint Lock")
            device.set_software_version("0.1.0")

            self.switch = mha.HASwitch("lock")
            self.switch.set_current_state(True)
            self.switch.set_name("lock servo")
            self.switch.set_icon("mdi:lock")
            self.switch.on_command(self.on_switch_command)
            mqtt.begin(BROKER_ADDR, user="user", password="passwd")
            self.loop_status = True
            thread.start_new_thread(self.begin_mqtt_loop, ())

    def begin_mqtt_loop(self):
        time.sleep(1)
        while self.loop_status:
            self.mqtt.loop()
            time.sleep_ms(200)

    def on_switch_command(self, sender: mha.HASwitch, state: bool):
        print("Switch state:", state)
        sender.set_state(state)
        if state:
            self.open_lock()
        else:
            self.close_lock()

    def open_lock(self):
        print("Opening lock...")
        self.servo.write_angle(110)
        if self.switch is not None:
            self.switch.set_state(True)

    def close_lock(self):
        print("Closing lock...")
        self.servo.write_angle(0)
        if self.switch is not None:
            self.switch.set_state(False)




def connect_to_ha_with_mqtt():


    device = mha.HADevice("001122AABBC0")  # (binascii.hexlify(machine.unique_id()).decode('utf-8'))
    mqtt = mha.HAMqtt(device)

    device.set_name("MHA Binary Sensor")
    device.set_software_version("0.1.0")

    global switch
    switch = mha.HASwitch("my_switch")
    switch.set_current_state(True)
    switch.set_name("My Switch")
    switch.set_icon("mdi:lightbulb")

    def on_switch_command(sender: mha.HASwitch, state: bool):
        print("Switch state:", state)
        sender.set_state(state)
        # to some action here

    mqtt.begin(BROKER_ADDR, user="user", password="passwd")
    mqtt.loop()

def update_sensor_status(status: bool):
    sensor.set_state("ON" if status else "OFF")


def before():
    # f.after_low_energy()  # 可以在wake期间多次执行
    # led.stop_led()
    time.sleep(0.5)  # 让led的轮训执行一轮


def after():
    # pin_handler(f)
    led.light_led()
    time.sleep(1)


# 定义中断处理函数
def pin_handler(fig):
    print("pin handler be triggered")
    # before()
    fig.register_sensor_handler(None)

    le.fresh_wake_time()  # 刷新sleep时间，防止处理事件过程中进入sleep

    fig.power_on()
    fig.control_ibln(color='blue')
    ack, receive, = f.auto_search()
    if ack == code_ok:
        fig.control_ibln(color='green')
        print("open servo")
        # print("ack ok", ack, receive)
        servo.open_lock()
        # _thread.start_new_thread(show_voltage_ble, ())
        time.sleep(3)
        print("close servo")
        servo.close_lock()
    else:
        fig.control_ibln(color='red')
        time.sleep(1)
    fig.control_ibln(color='none')
    f.sleep()
    if module_low_energy:
        f.power_off()
    fig.register_sensor_handler(pin_handler)
    after()


def run_figer_lock():
    print("init lock resource")
    # 定义舵机控制对象
    global servo
    servo = ServoLock(22, use_ha)
    servo.close_lock()

    # 定义低功耗对象
    global le
    le = lem.LowEnergy(wake_time, before_sleep=before, after_sleep=after, after_timer_wake=None)
    # make v_sensor on
    machine.Pin(16, Pin.OUT).value(1)

    time.sleep(0.1)
    global f
    f = ZW101(1, 19, 18, vcc=17, sensor=27) # Pin 1-6: v_sensor touch_out VCC tx rx GND
    f.power_on()
    # 注册开锁事件，下电。
    f.before_low_energy()  # 注册唤醒触发器
    f.register_sensor_handler(pin_handler)
    f.sleep()
    f.power_off()
    # 启动低功耗
    global module_low_energy
    if module_low_energy:
        le.start()


def run():
    while True:
        code = input("input code:")
        if code == 'q':
            break
        if code == 's':
            f.sleep()
            f.power_off()
        if code == 'on':
            f.power_on()
        if code.startswith("r"):
            sp = code.split(" ")
            color = "red" if len(sp) != 2 else sp[1]
            f.control_ibln(color)
        if code == 'ai':
            f.auto_identify()
        if code == 'sys':
            f.read_sys_para()
            f.read_index_table()
        if code == 'ae':
            f.auto_enroll(16)


if __name__ == '__main__':
    run_figer_lock()
