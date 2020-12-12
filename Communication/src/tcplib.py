import socket
import sys
import pickle
import time
from threading import Lock

# Information on sockets: https://docs.python.org/3/library/socket.html
# Inspiration for this lib: https://pythonprogramming.net/pickle-objects-sockets-tutorial-python-3/

HEADERSIZE = 10

s_print_lock = Lock()

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



#############################################
# GENERIC DATA RECEIVING VIA SOCKET/CONNECTION
#############################################
"""
This is a simple forward function.
It will listen to an input port and forward the 
data to an output port.
It works with all data types (the receiver must 
know how to process it)

Arguments:

    sender_socket:
            socket of the sender
    receiver_socket:
            socket of the receiver

Returns:    
            Nothing
"""
def forward(connection_name, sender_socket, receiver_socket, verbose=False):
    if verbose: 
        with s_print_lock:
            print('INFO: starting:', connection_name, '\n\t', sender_socket, '\n\t', receiver_socket)
    while True:
        sender_socket.listen(1)
        # Wait for a connection
        sock_in, client_address_in = sender_socket.accept()
        if verbose: 
            with s_print_lock:
                print('INFO: Input Connection from', client_address_in)

        receiver_socket.listen(1)
        # Wait for a connection
        sock_out, client_address_out = receiver_socket.accept()
        if verbose: 
            with s_print_lock:
                print('INFO: Output Connection from', client_address_out)

        try:
            # Receive the data in small chunks and retransmit it
            while True:
                
                """
                # Unpickling the forwarded data
                message = receiveData(sock_in, verbose)
                sendData(sock_out, message, verbose)
                print('SERVER forwarded: ', message)
                """
                
                full_msg = sock_in.recv(2**12)
                sock_out.send(full_msg)
                if verbose:
                    with s_print_lock:
                        print("SERVER forwarded", full_msg)

        finally:
            # Clean up the connection
            if verbose:
                with s_print_lock:
                    print('INFO: closing sockets: \'', client_address_in, '\' and \'', client_address_out, '\'')
            
            sock_in.close()
            sock_out.close()