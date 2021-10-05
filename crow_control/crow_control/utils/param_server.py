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
from zmq.sugar.constants import ROUTER_MANDATORY


class ParamServer():

    def __init__(self, start_port=25652, addr="127.0.0.1", protocol="tcp"):
        """ Creates a server for parameters. ParamClient nodes can subscribe to parameter updates
        handled via this server. If one client changes a parameter, other clients receive and update.

        Args:
            start_port (int, optional): A port on which this server operates. This is a port on which
            only the publisher will send parameter updates. Client parameter updates and definitions/declarations
            will be done over ports with numbers +3 and +7 higher. Defaults to 25652.
            addr (str, optional): IP address on which this server operates. See PyZMQ documentation
            for various options (e.g. "localhost", "*", etc.). Defaults to "127.0.0.1".
            protocol (str, optional): Protocol to use for communication. Probably don't change this,
            unless you really know what you are doing. Defaults to "tcp".
        """
        self.__context = zmq.Context()

        # socket to publish parameters
        self.__addr_pub = f"{protocol}://{addr}:{str(start_port)}"
        self.publisher = self.__context.socket(zmq.XPUB)
        self.publisher.setsockopt(zmq.XPUB_VERBOSE, True)
        self.publisher.bind(self.__addr_pub)

        # socket for param change requests
        self.__addr_ch = f"{protocol}://{addr}:{str(start_port + 3)}"
        self.changer = self.__context.socket(zmq.REP)
        self.changer.bind(self.__addr_ch)

        # socket for param definition
        self.__addr_def = f"{protocol}://{addr}:{str(start_port + 7)}"
        self.definer = self.__context.socket(zmq.REP)
        self.definer.bind(self.__addr_def)

        self.poller = zmq.Poller()
        self.poller.register(self.publisher, zmq.POLLIN)
        self.poller.register(self.changer, zmq.POLLIN)
        self.poller.register(self.definer, zmq.POLLIN)

        self.__params = {}
        self.active = True

        self.poller_thread = Thread(target=self.__poll, daemon=True)
        self.poller_thread.start()


    def define(self, param, default_value=None):
        """Create a definition for the given parameter. This is not necessary,
        the clients themselves can define/declare the parameters.

        Args:
            param (str): The parameter name.
            default_value (any, optional): Value to which the parameter should be set to. Defaults to None.
        """
        if param in self.__params:
            return
            # warn(f"Parameter {param} aleardy defined and somebody tries to redefine it!")

        setattr(ParamServer, param, property(
                lambda self, param=param: self.__params[param],
                lambda self, value, param=param: self.__set_param(param, value)
            ))

        self.__set_param(param, default_value)

    def destroy(self):
        """Stop the threaded poller and close all connections.
        """
        self.active = False
        self.publisher.close()
        self.changer.close()
        self.definer.close()

    def __set_param(self, param, value):
        self.__params[param] = value
        self.__broadcast_param(param)

    def __broadcast_param(self, param):
        self.publisher.send_multipart([param.encode('utf-8'), cpl.dumps(self.__params[param])])

    def __poll(self):
        while self.active:
            try:
                socks = dict(self.poller.poll())
            except zmq.error.ZMQError:
                break

            if self.publisher in socks:  # handle new subscriptions
                msg = self.publisher.recv()
                sub, param = msg[0], msg[1:].decode()
                if sub == 1:
                    if param == "":
                        for p in self.__params:
                            self.__broadcast_param(p)
                    else:
                        if param in self.__params:
                            self.__broadcast_param(param)
                        else:
                            self.__params[param] = None
                print(f"Someone {'' if sub == 1 else 'un'}subscribed on {param}")

            if self.changer in socks:  # handle param changes
                try:
                    param, value = self.changer.recv_multipart()
                except BaseException as e:
                    print(f"Error changing a parameter!\n{e}")
                    continue
                param = param.decode()
                if param in self.__params:
                    # self.__params[param] = cpl.loads(value)
                    self.__set_param(param, cpl.loads(value))
                    self.changer.send(b"true")
                else:
                    self.changer.send(b"false")

            if self.definer in socks:  # handle new param definitions here
                param, value, overwrite = self.definer.recv_multipart()
                param = param.decode()
                if param in self.__params:
                    if ord(overwrite) == 1:
                        self.__set_param(param, cpl.loads(value))
                    self.definer.send(b"true")
                else:
                    self.define(param, cpl.loads(value))
                    self.definer.send(b"true")
