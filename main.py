from machine import Pin
import time

led = Pin("LED", Pin.OUT)
for i in range(2):
    led.on()
    time.sleep(0.25)
    led.off()
    time.sleep(0.25)
    
class Motor:
    def __init__(self, pins):
        self.__pins = list(map(lambda pin: Pin(pin, mode=Pin.OUT, value=0), pins))
        self.__pos = 0
        self.__phaseIndex = 0
        self.__phases = [1, 1 | 2, 2, 2 | 4, 4, 4 | 8, 8, 8 | 1]
        print(self.__pins)

    def MoveRelative(self, delta):
        delta = int(delta)
        target = self.__pos + delta
        incr = 1 if delta > 0 else -1
        oldPhase = 0 # assume all off
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
        
        # turn all off
        for pin in self.__pins:
            pin.off()

        print("Moved to %2d " % (self.__pos))

    def MoveAbsolute(self, value):
        MoveRelative(value - self.__pos)


motor = Motor([2,3,4,5])
for index in range(10):
    motor.MoveRelative(500)
    motor.MoveRelative(-1000)
    motor.MoveRelative(500)
    motor.MoveRelative(-100)
    motor.MoveRelative(+200)
    motor.MoveRelative(-100)
