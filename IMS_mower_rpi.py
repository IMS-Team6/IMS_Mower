import serial
from picamera import PiCamera
from time import sleep
import os
import math
from threading import Thread
import datetime
import bluetooth

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
        # self.direction += newDirection
        self.direction = newDirection
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

class ReceiveBluetooth:
    def __init__(self):
        self.running = True
        self.receivedMessage = False
        self.message = " "
    
    def terminate(self):
        self.running = False
    
    def run(self, client):
        while self.running:
            self.message = client.recv(1024)
            if len(self.message) == 0:
                # Lost connection
                global mode
                mode = "Automated"
                print("Lost connection to application...")
                self.terminate()
            else:
                self.receivedMessage = True

# Bluetooth init
def bluetoothInit():

    # We need to wait until Bluetooth init is done
    sleep(10)

    # Make device visible
    t = Thread(target = os.system, args=('sudo hciconfig hci0 piscan', ), daemon = 1)
    t.start()

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
    uuid = "b2eac802-a015-49b4-8fd4-08212f5b2853"

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
#Init bluetoothconnection
app_sock = bluetoothInit()
bt = ReceiveBluetooth()
threadBT = Thread(target=bt.run, args=(app_sock), daemon=1)

#Create sessionID
sessionID = datetime.datetime.now().strftime("%y%m%d%H%M%S")

#Init Camera
camera = PiCamera()
picNmbr = 0

#Init Positioning
pos = CalculatePosition()
direction = 0
speed = 2
# Send the first position to backend here (Session started at 0,0)

#Init connection to Mower
serUSB = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serUSB.reset_input_buffer()

#Variables used in statemachines
running = True #Variable keeping track of program running
global mode
mode = "Automated"
reversing = False #Variable keeping track if mower is reversing when manual
turning = False #Variable keeping track if mower is turning when manual

while running:
    try:
        if mode == "Automated":
            if serUSB.in_waiting > 0:
                line = serUSB.readline().decode('utf-8').rstrip()
                if line == 'S':
                    #Start motors
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    # direction = 0
                    threadPos.start()
                    #serUSB.write(b'A')

                elif line == 'O':
                    #Obstacle encountered
                    pos.terminate()
                    threadPos.join()
                    camera.start_preview()
                    sleep(2)
                    camera.capture('/home/pi/Desktop/images/image%s.jpg' % picNmbr)
                    picNmbr += 1
                    print('Picture captured')
                    # Send picture to backend
                    #serUSB.write(b'A')

                elif line == 'B':
                    #Out of bounds
                    pos.terminate()
                    threadPos.join()
                    #serUSB.write(b'A')  
                
                elif line[0] == 'T':
                    #Turn
                    #direction += int(line[1:-1])
                    direction = int(line[1:-1])
                    #serUSB.write(b'A')       
            
            #Check if there is a message waiting from bluetooth
            if bt.receivedMessage:
                if bt.message == "MANUAL":
                    pos.terminate()
                    threadPos.join()
                    serUSB.write(b'M')
                    mode = "Manual" 
                bt.receivedMessage = False

        elif mode == "Manual":
            #Check if there is a message waiting from bluetooth
            if bt.receivedMessage:                
                if bt.message == "STOP":
                    # stop
                    serUSB.write(b'0')
                    pos.terminate()
                    threadPos.join()
                    #If the mower were reversing in previous state
                    if reversing:
                        pos.direction += 180
                        reversing = False
                    #If mower were turning in previous state
                    elif turning:
                        while True:
                            if serUSB.in_waiting > 0:
                                line = serUSB.readline().decode('utf-8').rstrip()
                                # direction += float(line)
                                direction = float(line)
                                serUSB.write(b'A') 
                                turning = False
                                break          
                elif bt.message == "FORWARD":    
                    # forward
                    serUSB.write(b'1')
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    threadPos.start()
                    # direction = 0               
                elif bt.message == "REVERSE":  
                    # backward      
                    pos.direction += 180
                    reversing = True
                    serUSB.write(b'2')
                    threadPos = Thread(target=pos.run, args=(speed, direction), daemon=1)
                    threadPos.start()
                    # direction = 0                   
                elif bt.message == "LEFT":    
                    # turnLeft
                    serUSB.write(b'3')
                    turning = True
                elif bt.message == "RIGHT":
                    # turnRight
                    serUSB.write(b'4')
                    turning = True
                elif bt.message == "AUTO":    
                    # changeMode
                    serUSB.write(b'5')
                    mode = "Automated"
                
                bt.receivedMessage = False  
    
    except KeyboardInterrupt:
            if app_sock is not None:
                app_sock.close()
            server_sock.close()
            print("Server going down")
            running = False   