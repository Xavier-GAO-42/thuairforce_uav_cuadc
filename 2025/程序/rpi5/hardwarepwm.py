# https://gist.github.com/Gadgetoid/b92ad3db06ff8c264eef2abf0e09d569
# use hardware PWM on pi5
from rpi_hardware_pwm import HardwarePWM
import time

def initPWM(FREQ,DUTY):
    global pwm
    pwm = HardwarePWM(pwm_channel=2, hz=FREQ, chip=0)
    pwm.start(DUTY) # full duty cycle

def setPWM(FREQ,DUTY):
    global pwm
    pwm.change_duty_cycle(DUTY)
    pwm.change_frequency(FREQ)

def stopPWM():
    global pwm
    pwm.stop()

if __name__ == "__main__":
    initPWM(10,20)
    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        stopPWM()
