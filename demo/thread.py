import _thread as thread
import time

loop = True
output=""
def one():
    while loop:
        print(output, end="")
        time.sleep(1)

def run():
    while True:
        inp=input("cmd: ")
        if inp == "quit":
            break
        if inp == "r":
            thread.start_new_thread(one, ())
        if inp.startswith("multi"):
            _, cnt = inp.split(" ")
            for i in range(int(cnt)):
                thread.start_new_thread(one, ())