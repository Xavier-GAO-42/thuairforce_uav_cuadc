# https://gist.github.com/Gadgetoid/b92ad3db06ff8c264eef2abf0e09d569
# use hardware PWM on pi5
# source ./CUADC/bin/activate
from rpi_hardware_pwm import HardwarePWM
import time


def initPWM(FREQ,DUTY):
    global pwm
    pwm = HardwarePWM(pwm_channel=3, hz=FREQ, chip=0)
    pwm.start(DUTY) # full duty cycle

def setPWM(FREQ,DUTY):
    global pwm
    pwm.change_duty_cycle(DUTY)
    pwm.change_frequency(FREQ)

def stopPWM():
    global pwm
    pwm.stop()

def toudan():
    initPWM(50,10)
    time.sleep(0.3)
    setPWM(50,5)
    time.sleep(0.1)
    setPWM(50,0)
    stopPWM()
if __name__ == "__main__":
    toudan()