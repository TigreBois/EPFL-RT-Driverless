import socket
import sys
import pickle

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