import socket
import sys
import pickle
import time
import tcplib

IP_PORT_IN =  10000
IP_PORT_OUT = 10001
IP_ADDR = 'localhost'
MY_MODULE_NAME = 'my module name' # Please enter here an arbitrary name for your code, which will help in logging

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
# INITIALIZE YOUR MODULE HERE
######################################################

# Create a TCP/IP socket for the input
sock_input = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, IP_PORT_IN)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_input.connect(server_address_in)

# Create a TCP/IP socket for the output
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, IP_PORT_OUT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)

module_running = True

try:
    while module_running:

        ######################################################
        # WHEN YOU ARE READY, START READING INCOMING DATA
        # TO PROCESS IT. THIS FUNCTION CALL IS BLOCKING,
        # IT WILL WAIT UNTIL THERE'S DATA. THE DATA WILL 
        # BE WAITING AND PILE UP UNTIL YOU READ IT.
        ######################################################
        msg = tcplib.receiveData(sock_input)
        print('message:', msg)


        ######################################################
        # PERFORM SOME CALCULATIONS.
        # WHEN YOU GET A RESULT YOU WISH TO SEND,
        # CONTINUE WITH THE CODE BELOW
        ######################################################
        my_results = ['my', 'awesome', 'results']
        tcplib.sendData(sock_output, my_results)


        ######################################################
        # IF YOU'RE DONE HANDLING THE DATA, THE LOOP
        # WILL TAKE YOU BACK TO WAITING ON INCOMING DATA.
        # IF YOUR MODULE ENCOUNTERS AN ERROR OR NEEDS TO 
        # TERMINATE, CONTINUE WITH THE CODE BELOW
        ######################################################
        if condition:
            module_running = False

finally:
    sock_input.close()
    sock_output.close()