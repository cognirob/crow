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


class ServiceClient():
    REQUEST_TIMEOUT = 3000  # in ms (how long to wait to send the request - doesn't do much...)
    RESPONSE_TIMEOUT = 3000  # in ms (how long to wait to receive a response)

    def __init__(self, port=242424, addr="127.0.0.1", protocol="tcp"):
        self.__context = zmq.Context()  # ZMQ context
        self.__addr = f"{protocol}://{addr}:{str(port)}"  # full address ~ sort of like a service name/identifier
        self._connect()

    def _connect(self):
        """Create a new ZMQ client, connect to the service server and set socket options
        """
        self._zmq_client = self.__context.socket(zmq.REQ)
        self._zmq_client.connect(self.__addr)
        self._zmq_client.setsockopt(zmq.SNDTIMEO, self.REQUEST_TIMEOUT)
        self._zmq_client.setsockopt(zmq.RCVTIMEO, self.RESPONSE_TIMEOUT)

    def _reconnect(self):
        """Reconnect after error (e.g., service timeout) otherwise socket in weird state = will not work
        """
        self._zmq_client.disconnect(self.__addr)
        self._connect()

    def destroy(self):
        self._zmq_client.close()

    def call(self, request):
        try:
            # 1) pickle and send the request
            self._zmq_client.send(cpl.dumps(request))
        except zmq.Again:  # timeout when sending (should not happen, unless ZMQ error)
            self._reconnect()
            return None
        try:
            # 2) wait and receive the response
            result = self._zmq_client.recv()
        except zmq.Again:  # response did not arrive in time
            self._reconnect()
            return None
        # unpickle and return the response
        return cpl.loads(result)
