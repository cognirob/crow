#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 26 14:56:16 2021

@author: syxtreme
"""

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
from zmq.utils.strtypes import asbytes


class ParamClient():
    
    def __init__(self, start_port=25652, addr="127.0.0.1", protocol="tcp"):
        self.__context = zmq.Context()
            
        # subscirbe to parameter changes
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.subscriber = self.__context.socket(zmq.SUB)
        self.subscriber.connect(self.__addr_pub)
        
        # socket for param change requests
        self.__addr_ch = f"{protocol}://{addr}:{str(start_port + 3)}"
        self.changer = self.__context.socket(zmq.REQ)
        self.changer.connect(self.__addr_ch)
        
        # socket for param definition
        self.__addr_def = f"{protocol}://{addr}:{str(start_port + 7)}"
        self.definer = self.__context.socket(zmq.REQ)
        self.definer.connect(self.__addr_def)
        
        self.__params = {}
        self.active = True
        
        self.poller_thread = Thread(target=self.__poll, daemon=True)
        self.poller_thread.start()

    def subscribe(self, param):
        if param in self.__params:
            return
        
        self.__params[param] = None
        setattr(ParamClient, param, property(
                lambda self, param=param: self.__params[param],
                lambda self, value, param=param: self.__set_param(param, value)
            ))
        self.subscriber.setsockopt(zmq.SUBSCRIBE, param.encode('utf-8'))
        
    def declare(self, param, default_value=None):
        self.__send_definition(param, default_value, 0)
        
    def define(self, param, value):
        self.__send_definition(param, value, 1)
        
    def __send_definition(self, param, value, overwrite):
        if param in self.__params:
            return
        
        self.definer.send_multipart([param.encode('utf-8'), cpl.dumps(value), asbytes(chr(overwrite))])
        response = self.definer.recv().decode()
        if response == "false":
            raise AttributeError(f"Could not declare the parameter {param} (value={value}).")

        self.subscribe(param)
        
    def destroy(self):
        self.active = False
        self.subscriber.close()
        self.changer.close()
        self.definer.close()
        
    def __set_param(self, param, value):
        self.changer.send_multipart([param.encode('utf-8'), cpl.dumps(value)])
        response = self.changer.recv().decode()
        # print(response)
        if response == "true":
            self.__params[param] = value
    
    def __poll(self):
        while self.active:
            param, msg = self.subscriber.recv_multipart()
            # print(param, msg)
            self.__params[param.decode()] = cpl.loads(msg)
        



