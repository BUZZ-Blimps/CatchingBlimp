
import errno
import fcntl, os
import random
import socket
import sys
import time

localIP     = "192.168.0.222"
localPort   = 20001
bufferSize  = 1024

msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
fcntl.fcntl(UDPServerSocket, fcntl.F_SETFL, os.O_NONBLOCK)

clientDataTime = time.time()

#x, y, radius, area, confidence
clientAddr = None

print("UDP server up and listening")

#Wait for message from client
while(True):
    try:
        #Check for incoming messages
        msg = UDPServerSocket.recvfrom(bufferSize)

        #If we received a message, log the IP address
        message = msg[0]
        clientAddr = msg[1]

        printMsg = "Received Message from Client:{}".format(message)
        printAddr  = "Client IP Address:{}".format(clientAddr)
        # print(printMsg)
        # print(printAddr)
    except socket.error as e:
        err = e.args[0]
        if err != errno.EAGAIN and err != errno.EWOULDBLOCK:
            # a "real" error occurred
            print(e)
            sys.exit(1)

    now = time.time()
    if (now - clientDataTime >= 1.0) and (clientAddr is not None):

        outMsg = ""

        for i in range(2):
            if i == 0:
                objType = "ball"
            else:
                objType = "goal"
            x = random.random()*100
            y = random.random()*100
            r = random.random()*100
            a = random.random()*100
            c = random.random()*100
            objStr = "{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f};".format(objType, x, y, r, a, c)
            outMsg += objStr
        UDPServerSocket.sendto(str.encode(outMsg), clientAddr)
        print("Sent message to ", clientAddr)
        clientDataTime = now

# Sending a reply to client
# 
    