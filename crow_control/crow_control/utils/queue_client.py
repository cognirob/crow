#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 26 14:56:16 2021

@author: syxtreme
"""

import zmq
from threading import Thread
from warnings import warn
import cloudpickle as cpl
from zmq.utils.strtypes import asbytes


class QueueClient():

    def __init__(self, queue_name="queue", start_port=12821, addr="127.0.0.1", protocol="tcp"):
        self.__queue_name = queue_name.encode('utf-8')
        self.__context = zmq.Context()

        # subscirbe to parameter changes
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.subscriber = self.__context.socket(zmq.SUB)
        self.subscriber.connect(self.__addr_pub)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, self.__queue_name)

        self.update_cb = None
        self.pop_cb = None
        self.last_value_cache = None
        self.last_pop_cache = None
        self.active = True

        self.poller_thread = Thread(target=self.__poll, daemon=True)
        self.poller_thread.start()

    def hook_on_update(self, callback):
        self.update_cb = callback

    def hook_on_pop(self, callback):
        self.pop_cb = callback

    def destroy(self):
        self.active = False
        self.subscriber.close()

    def __poll(self):
        while self.active:
            topic, msg = self.subscriber.recv_multipart()
            # print(topic)
            if b'update' in topic:
                self.last_value_cache = cpl.loads(msg)
                if self.update_cb is not None:
                    self.update_cb(self.last_value_cache)
            elif b'pop' in topic:
                self.last_pop_cache = cpl.loads(msg)
                if self.pop_cb is not None:
                    self.pop_cb(self.last_pop_cache)




