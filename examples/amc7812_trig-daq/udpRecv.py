from socket import *
import time
import csv

#address = ( '192.168.1.183', 5000) #Defind who you are talking to (must match arduino IP and port)
address = ( '169.254.5.12', 5000) #Defind who you are talking to (must match arduino IP and port)
client_socket = socket(AF_INET, SOCK_DGRAM) #Set Up the Socket
client_socket.settimeout(1) #only wait 1 second for a resonse
count=0

client_socket.sendto("start", address) #send command to arduino
print "{"
try:
    #while(count<10): #Main Loop
    while(1): #Main Loop
        rec_data, addr = client_socket.recvfrom(2048) #Read response from arduino
        print rec_data #Print the response from Arduino
        count+=1
except KeyboardInterrupt:
    pass
print "}"
client_socket.sendto("stop", address) #send command to arduino
client_socket.close()
