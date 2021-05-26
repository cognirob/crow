#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 25 14:19:55 2021

@author: syxtreme
"""

import zmq
from threading import Thread
from warnings import warn
import cloudpickle as cpl
from collections import deque


class QueueServer():
    
    def __init__(self, maxlen=-1, queue_name="queue", start_port=12821, addr="127.0.0.1", protocol="tcp"):
        self.__queue_name = queue_name.encode('utf-8')
        self.__context = zmq.Context()
            
        # socket to publish parameters
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.__publisher = self.__context.socket(zmq.PUB)
        self.__publisher.bind(self.__addr_pub)
        self.buffer = deque(maxlen=maxlen)
        
    def append(self, data):
        self.buffer.append(data)
        self.__broadcast()

    def pop(self):
        data = self.buffer.pop()
        self.__broadcast()
        self.__broadcast_popped(data)
        return data
    
    def remove(self, index):
        del self.buffer[index]
        self.__broadcast()        

    def destroy(self):
        del self.buffer
        self.publisher.close()
              
    def __broadcast(self):
        self.__publisher.send_multipart([self.__queue_name + b'.update', cpl.dumps(self.buffer)])
        
    def __broadcast_popped(self, data):
        self.__publisher.send_multipart([self.__queue_name + b'.popped', cpl.dumps(data)])
        
    def __len__(self):
        return len(self.buffer)
        
