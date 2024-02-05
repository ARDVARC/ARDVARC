from socket import *

def read_uplink_data():

    # socket stuff
    try:
        read_from_socket()
    except timeout:
        log_error()
        return -1
            
    
def read_from_socket():
    pass

def log_error():
    pass

