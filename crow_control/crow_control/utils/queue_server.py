#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 25 14:19:55 2021

@author: syxtreme
"""

import zmq
from threading import Thread, RLock
from warnings import warn
import cloudpickle as cpl
from collections import deque


class QueueServer():

    def __init__(self, maxlen=-1, queue_name="queue", start_port=12821, addr="127.0.0.1", protocol="tcp"):
        self.__queue_name = queue_name.encode('utf-8')
        self.__context = zmq.Context()
        self._rlock = RLock()

        # socket to publish parameters
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.__publisher = self.__context.socket(zmq.PUB)
        self.__publisher.bind(self.__addr_pub)
        self.buffer = deque(maxlen=maxlen)

    def append(self, data):
        self._rlock.acquire()
        self.buffer.append(data)
        self.__broadcast()
        self._rlock.release()

    def pop(self):
        try:
            self._rlock.acquire()
            data = self.buffer.popleft()
        except IndexError as ie:
            # print("buffer empty")
            raise IndexError(self.buffer)
        else:
            self.__broadcast()
            self.__broadcast_popped(data)
            return data
        finally:
            self._rlock.release()

    def remove(self, index):
        self._rlock.acquire()
        del self.buffer[index]
        self.__broadcast()
        self._rlock.release()

    def destroy(self):
        self._rlock.acquire()
        del self.buffer
        self.__publisher.close()
        self._rlock.release()

    def find_name_index(self, name):
        for pos, t in enumerate(self.buffer):
            if t[2] == name:
                return pos
        return None

    def __broadcast(self):
        self.__publisher.send_multipart([self.__queue_name + b'.update', cpl.dumps(self.buffer)])

    def __broadcast_popped(self, data):
        print("#####################")
        print(data)
        self.__publisher.send_multipart([self.__queue_name + b'.popped', cpl.dumps(data)])

    def __len__(self):
        return len(self.buffer)

