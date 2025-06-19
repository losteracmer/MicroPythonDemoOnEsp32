# 记录

设置一个别名，方便调用
```shell
alias mp=mpremote

mp resume + cp fingerprint_lock.py :/

```

## 常用命令

* 批量copy文件
```shell
mpremote fs -r cp lib/ :/lib/
```

* resume 的含义：
```python
def do_resume(state, _args=None):
    state._auto_soft_reset = False
```

* 修改波特率 或者
```python
# transport_serial.py

class SerialTransport(Transport):
    def __init__(self, device, baudrate=38400, wait=0, exclusive=True, timeout=1):
        self.in_raw_repl = False
        self.use_raw_paste = True
        self.device_name = device
        self.mounted = False
```
