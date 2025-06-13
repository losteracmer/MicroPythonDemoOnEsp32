import network
import ubinascii

def get_display_width(s):
    """计算字符串的显示宽度（中文算2个字符，英文算1个字符）"""
    width = 0
    for c in s:
        width += 2 if ord(c) > 255 else 1
    return width

def wifi_scan():
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    
    print("\n正在扫描WiFi...")
    nets = sta.scan()
    
    # 表头
    header_name = "WiFi名称"
    header_mac = "MAC地址"
    header_strength = "信号强度"
    
    # 计算各列最大宽度
    max_name_width = max([get_display_width(net[0].decode('utf-8')[:16]) 
                         for net in nets] + [get_display_width(header_name)])
    max_mac_width = max([len("00:00:00:00:00:00"), get_display_width(header_mac)])
    
    print("\n扫描结果：")
    # 打印表头
    print("{:<{}} {:<{}} {}".format(
        header_name, max_name_width,
        header_mac, max_mac_width,
        header_strength
    ))
    print("-" * (max_name_width + max_mac_width + 10))
    
    for net in nets:
        ssid = net[0].decode('utf-8') or "<隐藏网络>"
        bssid = ":".join(ubinascii.hexlify(net[1]).decode('utf-8')[i:i+2] 
                for i in range(0, 12, 2)).upper()
        rssi = net[3]
        
        # 信号强度图标
        strength = (
            "████" if rssi > -60 else  # 强
            "███ " if rssi > -70 else  # 中强
            "██  " if rssi > -80 else  # 中弱
            "█   "                     # 弱
        )
        
        # 限制SSID显示长度
        display_ssid = ssid[:16] + ".." if get_display_width(ssid) > 16 else ssid
        
        # 格式化输出
        print("{:<{}} {:<{}} {} ({}dBm)".format(
            display_ssid, max_name_width,
            bssid, max_mac_width,
            strength,
            rssi
        ))
    
    print("=" * (max_name_width + max_mac_width + 10))
    print("注：信号强度 ████=强 | ███=中 | ██=弱 | █=很差")
    sta.active(False)

