from machine import Pin, I2C
import time
from pico_i2c_lcd import I2cLcd

led = Pin("LED")
led.off()
resetButton = Pin(17, Pin.IN, Pin.PULL_UP)
runButton = Pin(21, Pin.IN, Pin.PULL_UP)

i2c = I2C(0, sda=Pin(0), scl=Pin(1))
lcd = I2cLcd(i2c, 63, 2, 16)
devices = i2c.scan()

def resetButtonPressed():
  return False if resetButton.value() else True

def runButtonPressed():
  return False if runButton.value() else True

def show(message):
    lcd.move_to(0, 1)
    lcd.putstr(message)
    
class Motor:
    def __init__(self, pins):
        self.__pins = list(map(lambda pin: Pin(pin, mode=Pin.OUT, value=0), pins))
        self.__pos = 0
        self.__phaseIndex = 0
        self.__phases = [1, 1 | 2, 2, 2 | 4, 4, 4 | 8, 8, 8 | 1]
        
    def Show(self):
        lcd.move_to(0, 0)
        lcd.putstr(f"{self.__pos:09}")
    
    def Reset(self):
        self.__pos = 0
    
    def MoveRelative(self, delta):
        show('run  ')
        delta = int(delta)
        target = self.__pos + delta
        incr = 1 if delta > 0 else -1
        oldPhase = 0 # assume all off
        led.on()
        while self.__pos != target and runButtonPressed():
            phaseIndex = self.__phaseIndex + incr
            newPhase = self.__phases[phaseIndex & 7]
            
            # compute "off" - should disable before we enable to prevent 3 being powered
            delta = oldPhase & ~newPhase
            if delta & 1: self.__pins[0].off()
            if delta & 2: self.__pins[1].off()
            if delta & 4: self.__pins[2].off()
            if delta & 8: self.__pins[3].off()
            # compute "on"
            delta = newPhase & ~oldPhase
            if delta & 1: self.__pins[0].on()
            if delta & 2: self.__pins[1].on()
            if delta & 4: self.__pins[2].on()
            if delta & 8: self.__pins[3].on()
            #book-keeping
            self.__phaseIndex = phaseIndex
            self.__pos = self.__pos + incr
            oldPhase = newPhase
            time.sleep(0.001)
            
        # turn all off
        for pin in self.__pins:
            pin.off()
        led.off()
        print("Moved to %2d " % (self.__pos))
        self.Show()
        show('stop ')

    def MoveAbsolute(self, value):
        MoveRelative(value - self.__pos)

motor = Motor([2,3,4,5])

show('init ')
while True:
    if resetButtonPressed():
        motor.Reset()
        show('reset')
    elif runButtonPressed():
        motor.MoveRelative(10000000)
        show('run  ')
    time.sleep(0.05)
    motor.Show()
    
