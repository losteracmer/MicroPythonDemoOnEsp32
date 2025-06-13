import network
import machine
import utime
import _thread
import webrepl

"""
开机时按住pin4，自动打开AP + web repl，方便调试
"""

# 配置参数
TOUCH_PIN = 4  # 使用的Touch引脚（根据实际连接修改）
THRESHOLD = 3  # 触发阈值（5秒内3次）
AP_SSID = "ESP32-AP"
AP_PASSWORD = "webrepl123"
WEBREPL_PWD = "0000"  # WebREPL密码


def setup_ap_mode():
    """开启AP模式并启动WebREPL"""
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=AP_SSID)
    ap.active(True)

    # 启动WebREPL
    webrepl.start(password=WEBREPL_PWD)
    print(f"\nWebREPL已启动！")
    print(f"AP热点: {AP_SSID}, 密码: {AP_PASSWORD}")
    print(f"WebREPL地址: ws://{ap.ifconfig()[0]}:8266/")
    print(f"WebREPL密码: {WEBREPL_PWD}")


def touch_detector():
    """Touch检测线程"""
    touch = machine.TouchPad(machine.Pin(TOUCH_PIN))
    trigger_count = 0
    last_trigger_time = utime.ticks_ms()

    while True:
        if touch.read() >  100000:  # 检测Touch（阈值根据实际调整）
            trigger_count += 1
            print(f"Touch触发 #{trigger_count}")
            last_trigger_time = utime.ticks_ms()
            utime.sleep_ms(300)  # 防抖延迟

        # 检查是否超时
        if utime.ticks_diff(utime.ticks_ms(), last_trigger_time) > 5000:
            trigger_count = 0
            print("un tiger repl")
            break

        # 达到触发条件
        if trigger_count >= THRESHOLD:
            print("满足触发条件，启动AP+WebREPL...")
            setup_ap_mode()
            break

        utime.sleep_ms(100)

def trigger_repl():
    # debug
    print("start load repl")
    _thread.start_new_thread(touch_detector, ())