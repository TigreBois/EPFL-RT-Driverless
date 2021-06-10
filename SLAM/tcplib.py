import socket
import pickle
from .slam import SLAM

#Credit: A. Unnervik

# Information on sockets: https://docs.python.org/3/library/socket.html
# Inspiration for this lib: https://pythonprogramming.net/pickle-objects-sockets-tutorial-python-3/

HEADERSIZE = 10

#############################################
# GENERIC DATA SENDING VIA SOCKET/CONNECTION
#############################################
"""
This is a simple generic data sending function.
It works with all data types (the receiver must 
know how to process it).
Arguments:
    socket: 
            socket of the destinator
    msg:  
            data packet you wish to send
"""
def sendData(dest, msg, verbose=False):
    p_msg = pickle.dumps(msg)                    # we pickle the data before sending it, for convenience
    
    # We prepend the pickled message with a fixed header 
    # which says how long the complete message is.
    # It's necessary because in a stream of data, you can't guess.
    data = bytes(f"{len(p_msg):<{HEADERSIZE}}", 'utf-8') + p_msg

    if verbose:
        print("INFO sendData() data:", data)

    ret = dest.send(data)

    if ret is None:
        print("ERROR sendData() encountered an error when sending data.")


#############################################
# GENERIC DATA RECEIVING VIA SOCKET/CONNECTION
#############################################
"""
This is a simple generic receive function.
It works with all data types (the receiver must 
know how to process it)
Arguments:
    socket:
            socket of the sender
Returns:    
            data packet you received
"""
# Note: For best match with hardware and network realities, the value of bufsize in recv(bufsize)
# should be a relatively small power of 2, for example, 4096. 
def receiveData(socket, verbose=False):
    data_size = socket.recv(HEADERSIZE)
    msglen = int(data_size)
    full_msg = socket.recv(msglen)

    if verbose:
        print("INFO receiveData() data_size:\t\t", data_size)
        print("INFO receiveData() msglen:\t\t", msglen)
        print("INFO receiveData() full_msg:\t\t", full_msg)
        print("INFO receiveData() full_msg unpickled:\t", pickle.loads(full_msg))

    return pickle.loads(full_msg)


IP_PORT_IN_PERCEPTION =  10020
IP_PORT_IN_MOTION =  10021
IP_PORT_OUT = 10022
IP_ADDR = 'localhost'
MY_MODULE_NAME = 'SLAM'

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
# INITIALIZE YOUR MODULE HERE
######################################################

# Create a TCP/IP socket for the perception input
sock_input_perception = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_perception = (IP_ADDR, IP_PORT_IN_PERCEPTION)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in_perception))
sock_input_perception.connect(server_address_in_perception)

# Create a TCP/IP socket for the motion estimation input
sock_input_motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_motion = (IP_ADDR, IP_PORT_IN_MOTION)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in_motion))
sock_input_motion.connect(server_address_in_motion)

# Create a TCP/IP socket for the output
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, IP_PORT_OUT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)

class TCPLib:
    def __init__(self) -> None:
        self.module_running = False

    def run_module(self):
        self.module_running = True
        
        try:
            while self.module_running:

                ######################################################
                # WHEN YOU ARE READY, START READING INCOMING DATA
                # TO PROCESS IT. THIS FUNCTION CALL IS BLOCKING,
                # IT WILL WAIT UNTIL THERE'S DATA. THE DATA WILL 
                # BE WAITING AND PILE UP UNTIL YOU READ IT.
                ######################################################
                cone_data   = receiveData(sock_input_perception)
                motion_data = receiveData(sock_input_motion)
                built_map   = SLAM.main(cone_data, motion_data)

                ######################################################
                # PERFORM SOME CALCULATIONS.
                # WHEN YOU GET A RESULT YOU WISH TO SEND,
                # CONTINUE WITH THE CODE BELOW
                ######################################################
                sendData(sock_output, built_map)

                ######################################################
                # IF YOU'RE DONE HANDLING THE DATA, THE LOOP
                # WILL TAKE YOU BACK TO WAITING ON INCOMING DATA.
                # IF YOUR MODULE ENCOUNTERS AN ERROR OR NEEDS TO 
                # TERMINATE, CONTINUE WITH THE CODE BELOW
                ######################################################
                # Need to figure out the terminating condition, but for let's pretend it never stops
                #if len(built_map) > 20:
                #    module_running = False

        except KeyboardInterrupt:
            print("Press Ctrl-C to terminate while statement")
            pass

        finally:
                sock_input_perception.close()
                sock_input_motion.close()
                sock_output.close()

    def stop_module(self):
        self.module_running = False

    

