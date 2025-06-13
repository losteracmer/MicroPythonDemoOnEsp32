import network
import urequests
import time

WIFI_SSID = "Avengers"
WIFI_PASSWORD = "qwertYUIOP"
URL = "http://www.qq.com"

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


def send_get_request(url):
    response = urequests.get(url)
    print("Response status code:", response.status_code)
    print("Response content:", response.text)
    response.close()



def run():
    while True:
        inp = input("cmd: ")
        if inp == "q":
            break
        if inp == "c":
            connect_wifi(WIFI_SSID, WIFI_PASSWORD)
        if inp == "r":
            send_get_request(URL)