from machine import Pin, I2C
import time
from pico_i2c_lcd import I2cLcd

perDay = 49100
perHour = perDay / 12
perMinute = perHour / 60
def Tick(hour, minute):
    hour = hour - 4 # we use 4pm as the init
    return int((hour * perHour) + (minute * perMinute))

scenes = [1, Tick(4,15), Tick(7, 20), Tick(9, 12)] # first value is current scene index

led = Pin("LED")
led.off()
backButton = Pin(18, Pin.IN, Pin.PULL_UP)
forwardButton = Pin(21, Pin.IN, Pin.PULL_UP)

i2c = I2C(0, sda=Pin(0), scl=Pin(1))
lcd = I2cLcd(i2c, 63, 2, 16)
devices = i2c.scan()

def backButtonPressed():
  return False if backButton.value() else True

def forwardButtonPressed():
  return False if forwardButton.value() else True
    
def Time(tick):
    totalMinutes = int(tick / perMinute)
    minute = totalMinutes % 60
    hour = (4 + (totalMinutes // 60)) % 12
    return f'{hour:02d}:{minute:02d}'

def Pad(message): # for LCD1602
    message = str(message)
    while len(message) < 16:
        message = message + " "
    return message

def ShowMessage(message):
    lcd.move_to(0, 1)
    lcd.putstr(Pad(message))
    
class Motor:
    def __init__(self, pins):
        self.__pins = list(map(lambda pin: Pin(pin, mode=Pin.OUT, value=0), pins))
        self.__pos = 0
        self.__phaseIndex = 0
        self.__phases = [1, 1 | 2, 2, 2 | 4, 4, 4 | 8, 8, 8 | 1]

    def Show(self):
        lcd.move_to(0, 0)
        lcd.putstr(Pad(Time(self.__pos)))
    
    def Reset(self):
        self.__pos = 0
    
    def MoveRelative(self, delta):
        delta = int(delta)
        
        final = Time(self.__pos + delta)
        hint = ">>>" if delta > 0 else "<<<"
        ShowMessage(f'{hint} {final}')  
        
        target = self.__pos + delta
        incr = 1 if delta > 0 else -1
        oldPhase = 0 # assume all off
        led.on()
        while self.__pos != target:
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
            
            if (phaseIndex % 100) == 0 and backButtonPressed() and forwardButtonPressed():
                break
            
        # turn all off
        for pin in self.__pins:
            pin.off()
        led.off()
        self.Show()
        ShowMessage(f"Scene {scenes[0]}")
        
        while backButtonPressed() or forwardButtonPressed():
            time.sleep(0.5) # allow time to get fingers off buttons

    def MoveAbsolute(self, value):
        self.MoveRelative(value - self.__pos)

motor = Motor([2,3,4,5])
motor.Show()


rtc = machine.RTC()
def GetMinute():
    return rtc.datetime()[5]

def ChangeScene(newScene):
    if newScene >= 1 and newScene < len(scenes):
        scenes[0] = newScene
        motor.MoveAbsolute(scenes[newScene])

ShowMessage('set & press btn')
while not (backButtonPressed() or forwardButtonPressed()):
    time.sleep(0.05)

ChangeScene(scenes[0])

lastMinute = GetMinute()
while True:
    time.sleep(0.05)
    if backButtonPressed():
        ChangeScene(scenes[0] - 1)
    elif forwardButtonPressed():
        ChangeScene(scenes[0] + 1)
    else:
        newMinute = GetMinute()
        if newMinute != lastMinute:
            motor.MoveRelative(perMinute)
    lastMinute = GetMinute()
    
