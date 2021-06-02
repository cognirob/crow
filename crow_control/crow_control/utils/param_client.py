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
import time


class ParamClient():

    def __init__(self, start_port=25652, addr="127.0.0.1", protocol="tcp"):
        """ Creates a client for parameters. The client can subscribe to various parameters
        and then it will receive updates about changes to these parameters.
        The parameters can be accessed as if they were properties of this client.
        ***WARNING***
        The property voodoo does changes to the base class itself. That means,
        using more clients in the same script will be buggy - all instances will have the same
        properties. Although, this should be ok in most cases, it is recommended to find another way.

        Args:
            start_port (int, optional): A port where the parameters server operates. This is a port on which
            only the publisher will send parameter updates. Parameter changes and definitions/declarations
            will be done over ports with numbers +3 and +7 higher. Defaults to 25652.
            addr (str, optional): IP address on which the server operates. Defaults to "127.0.0.1".
            protocol (str, optional): Protocol to use for communication.
        """
        self.__context = zmq.Context()

        # subscirbe to parameter changes
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.__subscriber = self.__context.socket(zmq.SUB)
        self.__subscriber.connect(self.__addr_pub)

        # socket for param change requests
        self.__addr_ch = f"{protocol}://{addr}:{str(start_port + 3)}"
        self.__changer = self.__context.socket(zmq.REQ)
        self.__changer.connect(self.__addr_ch)

        # socket for param definition
        self.__addr_def = f"{protocol}://{addr}:{str(start_port + 7)}"
        self.__definer = self.__context.socket(zmq.REQ)
        self.__definer.connect(self.__addr_def)

        self.__params = {}
        self.active = True

        self.poller_thread = Thread(target=self.__poll, daemon=True)
        self.poller_thread.start()

    def wait_for_param(self, param, timeout=0):
        # timeout DOES NOT WORK, yet
        msg = self.wait_receive_param(param, timeout)
        if msg is None:
            return False
        return True

    def wait_receive_param(self, param, timeout=0):
        # timeout DOES NOT WORK, yet
        self.__subscriber.setsockopt(zmq.SUBSCRIBE, param.encode('utf-8'))
        start = time.time()
        msg = None
        while True:
            rcv_param, data = self.__subscriber.recv_multipart()
            if rcv_param.decode() == param:
                msg = cpl.loads(data)
                print(param, msg)
                break
        self.__subscriber.setsockopt(zmq.UNSUBSCRIBE, param.encode('utf-8'))
        if msg is not None:
            return msg

    def subscribe(self, param):
        """Subscribe to parameter updates. This does not guarantee that the parameter
        exists and has any value. This only tells the server that this client
        wants to be notified if a parameter with the specified name changes.
        At the begining, the value of the parameter will be "None" and will remain so
        until this client receives value update for the parameter. This will occur
        rather quickly if the parameter exists on the server but this is not known
        at the time this method is called. This should be fine, just be aware of it.

        Args:
            param (str): The name of the parameter
        """
        if param in self.__params:
            return

        self.__params[param] = None
        setattr(ParamClient, param, property(
                lambda self, param=param: self.__params[param],
                lambda self, value, param=param: self.__set_param(param, value)
            ))
        self.__subscriber.setsockopt(zmq.SUBSCRIBE, param.encode('utf-8'))

    def declare(self, param, default_value=None, type=None, description=""):
        """Subsribes to parameter updates. This does roughly the same as
        subscribe but presents a default value to fill in if the parameter
        does not exist on the server. Use this if you want to always start with some
        (not None) value but you don't want to overwrite any existing values.
        E.g., you need the param to be False at the beginning but remain True
        if it was set to True by other client (with the subscribe method,
        it would not be False but None, if it was not defined, yet).

        Args:
            param (str): The name of the parameter
            default_value (any, optional): Default value for the parameter if it was not
            defined, yet. Defaults to None.
        """
        self.__send_definition(param, default_value, 0)

    def define(self, param, value, type=None, description=""):
        """Subscribes to parameter updates. Unlike subscribe or declare, this method always
        overwrites the current parameter value. That is, whether the parameter exists on the server
        or not, after calling this method, it will have value set to "value".
        Use this method for clients that should "own" the control over this parameter.

        Args:
            param (str): The name of the parameter.
            value (any): The starting or new value of the parameter. The parameter is always
            set to this value upon definition.
        """
        self.__send_definition(param, value, 1)

    def destroy(self):
        """Stops the updates and closes all connection.
        """
        self.active = False
        self.__subscriber.close()
        self.__changer.close()
        self.__definer.close()

    def __send_definition(self, param, value, overwrite):
        if param in self.__params:
            return

        self.__definer.send_multipart([param.encode('utf-8'), cpl.dumps(value), asbytes(chr(overwrite))])
        response = self.__definer.recv().decode()
        if response == "false":
            raise AttributeError(f"Could not declare the parameter {param} (value={value}).")

        self.subscribe(param)

    def __set_param(self, param, value):
        self.__changer.send_multipart([param.encode('utf-8'), cpl.dumps(value)])
        response = self.__changer.recv().decode()
        # print(response)
        if response == "true":
            self.__params[param] = value

    def __poll(self):
        while self.active:
            param, msg = self.__subscriber.recv_multipart()
            # print(param, msg)
            self.__params[param.decode()] = cpl.loads(msg)




