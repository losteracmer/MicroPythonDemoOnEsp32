from machine import ADC,Pin,DAC

DAC_PIN = 26
ADC_PIN = 32
# atten(衰减)取值
#ADC.ATTN_0DB：无衰减（100mV - 950mV）
#ADC.ATTN_2_5DB: 2.5dB 衰减 (100mV - 1250mV)
#ADC.ATTN_6DB: 6dB 衰减 (150mV - 1750mV
#ADC.ATTN_11DB: 11dB 衰减 (150mV - 2450mV)

"""
esp32 ADC引脚:
    ADC模块1：Pin32 - Pin39
    ADC模块2：Pin(0, 2, 4, 12 - 15, 25 - 27)
ADC模块2被WIFI使用，因此使用WIFI时避免使用这些端口

esp32 DAC引脚:
    25， 26

esp32S3 没有DAC引脚
"""

""" ADC操作 """
def adc_r():
    adc = ADC(Pin(ADC_PIN, Pin.IN, Pin.PULL_DOWN), atten=ADC.ATTN_11DB)
    # 此方法根据块的分辨率返回原始 ADC 值，例如，0-4095 表示 12 位分辨率
    print("adc.read() ", adc.read())

    # 返回 ADC 值，分辨率为 16 位分辨率，值为 0 ~ 65535
    print("adc.read_u16()", adc.read_u16())
    """
    此方法使用 ADC 的已知特性和每个封装的 eFuse 值（在制造期间设置）返回以微伏为单位的校准输入电压（衰减前）。返回值只有毫伏分辨率（即，始终是 1000 微伏的倍数）。

    校准仅在 ADC 的线性范围内有效。特别是，接地的输入将读取为高于 0 微伏的值。然而，在线性范围内，将获得比使用read_u16()常数和缩放结果更准确和一致的结果。
    """
    print("adc.read_uv()", adc.read_uv())

""" DAC 操作 """
def dac_w():
    # 引脚电平写入 8 位 DAC 范围为 0 ~ 255 (对应 0 - 3.3v)
    import time
    dac = DAC(DAC_PIN)
    for i in range(255):
        dac.write(i)
        time.sleep(0.01)

def run():
    while True:
        inp = input("cmd:")
        if inp == "dac":
            dac_w()
        if inp == "adc":
            adc_r()
        if inp == "q":
            break