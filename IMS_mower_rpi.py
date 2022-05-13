from glob import glob
from sqlite3 import DatabaseError
import serial
from picamera import PiCamera
from time import sleep
import os
import math
from threading import Thread
import datetime
import bluetooth
import requests
import json

def sendPositionRequest(x, y, sessionID, state, collisionFlag):
    print(sessionID)
    url = "http://3.72.195.76/api/session/" + sessionID

    payload = json.dumps({
    "robotState": state,
    "positions": {
        "posX": x,
        "posY": y
    },
    "collision": collisionFlag
    })
    headers = {
    'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)
    print(response.text)


def sendImageRequest(x,y):
    path = '/home/pi/Desktop/images/image.jpg'
    url = "http://3.72.195.76/api/session/" + sessionID

    payload={
    'posX': x,
    'posY': y
    }
    files=[
    ('collisionImg', ('image.jpg',open(path, 'rb'), 'image/jpg'))
    ]
    headers = {
    'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload, files=files)
    print(response.text)    


class CalculatePosition:
    def __init__(self):
        self._running = True
        self.x = 0
        self.y = 0
        self.direction = 0
        self.messagesPerSecond = 100.0
        self.collision = False
    
    def terminate(self):
        print("(%s, %s)" % (int(self.x), int(self.y)))
        #Send data to backend here instead of printing
        #sendPositionRequest(int(self.x), int(self.y), sessionID, "MOVING", self.collision)
        self.collision = False
        self._running = False

    #speed is about 8/34 m/s => 0.235m/s
    def run(self, speed, newDirection):
        self._running = True
        sendMessage = 0
        global sessionID
        # self.direction += newDirection
        self.direction = newDirection
        if self.direction > 360:
            self.direction -= 360
        while self._running:
            self.x += speed/self.messagesPerSecond * math.cos((math.pi/2.0) - math.radians(self.direction))
            self.y += speed/self.messagesPerSecond * math.sin((math.pi/2.0) - math.radians(self.direction))
            #Round values to whatever backend wants.
            sleep(1/self.messagesPerSecond)
            sendMessage += 1
            if sendMessage == self.messagesPerSecond:
                print("(%s, %s)" % (int(self.x), int(self.y)))
                #Send data to backend here instead of printing
                #sendPositionRequest(int(self.x), int(self.y), sessionID, "MOVING", False)


class ReceiveBluetooth:
    def __init__(self, client):
        self.client = client
        self.running = True
        self.receivedMessage = False
        self.command = 0
    
    def terminate(self):
        self.running = False
    
    def run(self):
        while self.running:
            if self.receivedMessage == False:
                dataBuffer = self.client.recv(4)
                if len(dataBuffer) == 0:
                    # Lost connection
                    # global mode
                    # mode = "Automated"
                    print("Lost connection to application...")
                    self.terminate()
                else:
                    self.command = int.from_bytes(dataBuffer, "big")
                    #print("BT: %s" % self.command)
                    self.receivedMessage = True

# Bluetooth init
def bluetoothInit():

    # We need to wait until Bluetooth init is done
    sleep(5)

    # Make device visible
    t = Thread(target = os.system, args=('sudo hciconfig hci0 piscan', ), daemon = 1)
    t.start()
    sleep(1)

    # Create a new server socket using RFCOMM protocol
    global server_sock
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    # Bind to any port
    server_sock.bind(("", bluetooth.PORT_ANY))
    # Start listening
    server_sock.listen(1)

    # Get the port the server socket is listening
    port = server_sock.getsockname()[1]

    # The service UUID to advertise
    uuid = "7be1fcb3-5776-42fb-91fd-2ee7b5bbb86d"

    # Start advertising the service
    bluetooth.advertise_service(server_sock, "IMSMowerGrp6",
                       service_id=uuid,
                       service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                       profiles=[bluetooth.SERIAL_PORT_PROFILE])
    
    print("Waiting for connection on RFCOMM channel %d" % port)
    client_sock = None

    # This will block until we get a new connection
    client_sock, client_info = server_sock.accept()
    print("Accepted connection from ", client_info)
    return client_sock

#Main function starts here

#Init connection to Mower
serUSB = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serUSB.reset_input_buffer()

#Init bluetoothconnection
app_sock = bluetoothInit()
bt = ReceiveBluetooth(app_sock)
threadBT = Thread(target=bt.run, daemon=1)
threadBT.start()

#Init Camera
camera = PiCamera()

#Init Positioning
pos = CalculatePosition()
direction = 0
speed = 20
threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)


#Create sessionID
global sessionID
sessionID = datetime.datetime.now().strftime("%y%m%d%H%M%S")
# Send the first position to backend here (Session started at 0,0)
#sendPositionRequest(int(pos.x), int(pos.y), sessionID, "START", False)

#Variables used in statemachines
running = True #Variable keeping track of program running
global mode
mode = "Manual"
reversing = False #Variable keeping track if mower is reversing when manual
turning = False #Variable keeping track if mower is turning when manual

while running:
    try:
        if mode == "Automated":  
            if serUSB.in_waiting > 0:
                line = serUSB.readline().decode('utf-8').rstrip()
                print(line)
                if line == 'S':
                    #Start motors
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    # direction = 0
                    threadPos.start()
                   # serUSB.write(b'A')

                elif line == 'O':
                    #Obstacle encountered
                    pos.collision = True
                    pos.terminate()
                    threadPos.join()
                    camera.start_preview()
                    sleep(2)
                    camera.capture('/home/pi/Desktop/images/image.jpg')
                    print('Picture captured')
                    # Send picture to backend here
                    # Also send position + obstacle occured?
                    #serUSB.write(b'A')

                elif line == 'B':
                    #Out of bounds
                    pos.terminate()
                    threadPos.join()
                    #serUSB.write(b'A')
                
                elif line == 'T':
                    #Turn
                    #serUSB.write(b'A')
                    while True:
                        if serUSB.in_waiting > 0:
                            angle = serUSB.readline().decode('utf-8').rstrip()
                            #print(angle)
                            break
                    #direction += int(line[1:-1])
                    direction = int(angle)
                    #serUSB.write(b'A')

            #Check if there is a message waiting from bluetooth
            if bt.receivedMessage:
                print("%s in automated...." % bt.command)
                if bt.command == 9:
                    # Change mode
                    if threadPos.is_alive:
                        pos.terminate()
                        threadPos.join()
                    serUSB.write(b'M')
                    mode = "Manual"
                bt.receivedMessage = False

        elif mode == "Manual":
            #Check if there is a message waiting from bluetooth
            if bt.receivedMessage == True:
                #print(bt.command)         
                if bt.command == 0:
                    # stop
                    serUSB.write(b'0')
                    if threadPos.is_alive:
                        pos.terminate()
                        threadPos.join()
                    #If the mower were reversing in previous state
                    if reversing:
                        direction += 180
                        reversing = False
                    #If mower were turning in previous state
                    elif turning:
                        while True:
                            if serUSB.in_waiting > 0:
                                line = serUSB.readline().decode('utf-8').rstrip()
                                if(line == 'O' or line == 'S' or line == 'B' or line == 'T'):
                                    pass
                                else:
                                    # direction += float(line)
                                    direction = int(line)
                                    #serUSB.write(b'A') 
                                    turning = False
                                    break          
                
                elif bt.command == 1:    
                    # forward
                    serUSB.write(b'1')
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    threadPos.start()
                    # direction = 0               
                
                elif bt.command == 2:    
                    # turnLeft
                    serUSB.write(b'2')
                    turning = True
                
                elif bt.command == 3:
                    # turnRight
                    serUSB.write(b'3')
                    turning = True

                elif bt.command == 4:  
                    # backward      
                    direction += 180
                    reversing = True
                    serUSB.write(b'4')
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    threadPos.start()
                    # direction = 0                       
                
                elif bt.command == 8:    
                    # changeMode
                    serUSB.write(b'5')
                    mode = "Automated"
                bt.receivedMessage = False
                 
    
    except KeyboardInterrupt:
        if app_sock is not None:
            app_sock.close()
        server_sock.close()
        print("Server going down")
        #sendPositionRequest(int(pos.x), int(pos.y), sessionID, "STOP", False)
        pos.terminate
        running = False   