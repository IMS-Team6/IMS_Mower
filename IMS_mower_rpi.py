import serial
from picamera import PiCamera
from time import sleep
import os
import math
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
        self.direction += newDirection
        if self.direction > 360:
            self.direction -= 360
        while self._running:
            self.x += speed/2 * math.cos((math.pi/2.0) - math.radians(self.direction))
            self.y += speed/2 * math.sin((math.pi/2) - math.radians(self.direction))
            #Round values to whatever backend wants.
            print(self.x)
            print(self.y)
            #Send data to backend here instead of printing.
            sleep(1/2)

threadBT = Thread(target = os.system, args=('sudo rfcomm watch hci0', ), daemon = 1)
threadBT.start()
sleep(15) #Wait for connection
serBT = serial.Serial('/dev/rfcomm0')

camera = PiCamera()
picNmbr = 0

c = CalculatePosition()
direction = 0
speed = 2
threadPos = Thread(target=c.run, args=(speed, direction), daemon=1)

serUSB = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serUSB.reset_input_buffer()

running = True
mode = "Automated"
reversing = False #Variable keeping track if mower is reversing when manual

while running:
    if mode == "Automated":
        if serUSB.in_waiting > 0:
            line = serUSB.readline().decode('utf-8').rstrip()
            if line == 'S':
                #Start motors
                direction = 0
                threadPos = Thread(target=c.run, args=(speed, direction), daemon=1)
                threadPos.start()
                serUSB.write(b'A')

            elif line == 'O':
                #Obstacle encountered
                c.terminate()
                threadPos.join()
                camera.start_preview()
                sleep(2)
                camera.capture('/home/pi/Desktop/images/image%s.jpg' % picNmbr)
                picNmbr += 1
                print('Picture captured')
                serUSB.write(b'A')

            elif line == 'B':
                #Out of bounds
                c.terminate()
                threadPos.join()
                serUSB.write(b'A')    
            
            elif line[0] == 'T':
                #Turn
                direction += float(line[1:])
                serUSB.write(b'A')       
        #Check if there is a message waiting from bluetooth
            #If there is one, it should be to swap mower-mode. Handle it.
            # c.terminate()
            # threadPos.join()
            # serUSB.write(b'M')
            # mode = "Manual" 
    elif mode == "Manual":
        #Check if there is a message waiting from bluetooth
            #If there is one, handle it
            # stop
                # serUSB.write(b'0')
                # c.terminate()
                # threadPos.join()
                # if reversing:
                    # c.direction += 180
                    # reversing = False
            # forward
                # serUSB.write(b'1')
                # threadPos = Thread(target=c.run, args=(speed, direction), daemon=1)
                # threadPos.start()
                # direction = 0
            # backward
                # c.direction += 180
                # reversing = True
                # serUSB.write(b'2')
                # threadPos = Thread(target=c.run, args=(speed, direction), daemon=1)
                # threadPos.start()
                # direction = 0
            # turnRight
                # serUSB.write(b'4')
                # while True:
                    # if serUSB.in_waiting > 0:
                        # line = serUSB.readline().decode('utf-8').rstrip()
                        # direction += float(line)
                        # break
            # turnLeft
                # serUSB.write(b'3')
                # while True:
                    # if serUSB.in_waiting > 0:
                        # line = serUSB.readline().decode('utf-8').rstrip()
                        # direction += float(line)
                        # break
            # changeMode
                # mode = "Automated"  
       
       
       
       
        """"
        match line:
            
            case 'Obstacle encountered':
                c.terminate()
                t.join()
                camera.start_preview()
                sleep(2)
                camera.capture('/home/pi/Desktop/images/image%s.jpg' % picNmbr)
                picNmbr += 1
                print('Picture captured')
            
            case line if line.startswith('Turn'):
                direction = float(line[5:])
                #ITSAFLOAT!!!!!
            
            case 'Run':
                t = Thread(target=c.run, args=(speed, direction), daemon=1)
                t.start()
            
            case 'Out of bounds':
                c.terminate()
                t.join()

        ser.write(b'Ack\n')   
        #SEND ACK TO ARDUINO, ARDUINO SHOULD WAIT FOR ACK BEFORE CONTINUING!!!!!
        #This enables the arduino to wait for the rpi to finish what its doing ex: taking picture.
        """