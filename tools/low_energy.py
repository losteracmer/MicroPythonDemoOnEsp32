import machine
import utime
import _thread
from machine import Pin


def wake_with(num):
    # esp32.wake_on_ext1(pins=(Pin(26),Pin(27)),level=esp32.WAKEUP_ALL_LOW)
    wake_pin = Pin(num, Pin.IN)
    wake_pin.irq(trigger=Pin.WAKE_HIGH, wake=machine.DEEPSLEEP)  # 配置唤醒中断


# 开线程，灵活的控制sleep状态
# 1. 常开模式，定时重启模式


class LowEnergy:
    status = None
    wake_duration = None
    last_wake_time = None
    before_sleep = None
    after_sleep = None
    after_timer_wake = None

    # wake_duration: second
    def __init__(self, wake_duration, before_sleep=None, after_sleep=None, after_timer_wake=None):
        self.last_wake_time = utime.time()
        self.wake_duration = wake_duration
        self.before_sleep = before_sleep
        self.after_sleep = after_sleep
        self.after_timer_wake = after_timer_wake

    def start(self, sleep_now: bool = False):
        # 启动一个线程，轮训上次唤醒时间，如果超过了wake_duration，则进入sleep
        if sleep_now:
            self.sleep_now()
        else:
            self.status = "sleeping"
            _thread.start_new_thread(self.sleep_check, ())

    def stop(self):
        self.status = "waking"

    def fresh_wake_time(self):
        self.last_wake_time = utime.time()

    def sleep_check(self):
        while True:
            # print("check sleeping")
            if self.status == "sleeping":
                if utime.time() - self.last_wake_time > self.wake_duration:
                    self.sleep_now()
            elif self.status == "waking":
                print("stop check loop sleeping")
                break
            utime.sleep(1)

    def sleep_now(self):
        if self.before_sleep is not None:
            self.before_sleep()

        print("ready to sleep...")
        utime.sleep_ms(1)  # 避免日志打印不完整
        machine.lightsleep()
        # wake up
        self.last_wake_time = utime.time()
        self.wake_reason()

    def wake_reason(self):
        if machine.reset_cause() == machine.SOFT_RESET:
            print('normal reboot')
            # do something
        elif machine.reset_cause() == machine.DEEPSLEEP_RESET:
            print('reboot from deepsleep')
        elif machine.reset_cause() == machine.HARD_RESET:
            print("hard reset")
        if machine.wake_reason() == machine.PIN_WAKE:
            print("pin wake")
            if self.after_sleep is not None:
                self.after_sleep()
        elif machine.wake_reason() == machine.EXT1_WAKE:
            print("ext1 wake")
        elif machine.wake_reason() == machine.TIMER_WAKE:
            print("timer wake")
            if self.after_timer_wake is not None:
                self.after_timer_wake()
