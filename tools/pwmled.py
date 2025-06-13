import machine
import time
import _thread

# 定义全局变量来控制呼吸灯的开关
running = True

# 初始化PWM
led = machine.Pin(21, machine.Pin.OUT)
pwm = machine.PWM(led)
pwm.freq(1000)  # 设置PWM频率为1kHz


def light_led():
    global running
    running = True


def stop_led():
    global running
    running = False


# 呼吸灯线程
def breath_led():
    global running
    while True:
        if running:
            # 渐亮
            for duty in range(0, 20):
                pwm.duty(duty)
                time.sleep_ms(10)
            # 渐暗
            for duty in range(20, -1, -1):
                pwm.duty(duty)
                time.sleep_ms(10)
        else:
            pwm.duty(0)  # 如果关闭，设置占空比为0
            time.sleep(0.4)  # 减少CPU占用


def start():
    # 启动呼吸灯线程
    _thread.start_new_thread(breath_led, ())
