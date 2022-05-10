import math
import time
from threading import Thread

class CalculatePosition:
    def __init__(self):
        self._running = True
        self.x = 0
        self.y = 0
        self.direction = 0
    
    def terminate(self):
        self._running = False

    #speed is about 8/34 m/s => 0.235m/s
    def run(self, speed, newDirection):
        self._running = True
        self.direction = newDirection
        if self.direction > 360:
            self.direction -= 360
        while self._running:
            self.x += speed/1.0 * math.cos((math.pi/2.0) - math.radians(self.direction))
            self.y += speed/1.0 * math.sin((math.pi/2.0) - math.radians(self.direction))
            print("x: %s" % int(self.x))
            print("y: %s\n" % int(self.y))
            time.sleep(1)

c = CalculatePosition()
direction = 0
speed = 2
t = Thread(target=c.run, args=(speed, direction), daemon=1)

while True:
    operation = input('Enter new operation: ')
    match operation: 
        case 'stop':
            c.terminate()
            t.join()
        case 'start':
            t = Thread(target=c.run, args=(speed, int(direction)), daemon=1)
            t.start()
        case 'turn':
            c.terminate()
            t.join()
            direction = input('Enter updated direction: ')
        case _: 
            print('invalid input')    







# t = Thread(target=c.run, args=(2, 0), daemon=1)
# t.start()
# for i in range(20):
#     print('x: %s, y: %s.' % (c.x, c.y))
#     time.sleep(1/5)
# c.terminate()
# t.join()

