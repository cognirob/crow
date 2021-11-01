from crow_ontology.utils import query_parser
from rclpy.node import Node
import sys
import time
from typing import Any, Dict, List, Set, Tuple
from textwrap import wrap
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import pandas as pd
import npyscreen
from crow_ontology.crowracle_client import CrowtologyClient
import curses
import inspect
import re
from itertools import chain
import os
from crow_ontology.utils import QueryParser
from npyscreen.wgmultiline import MultiLine
import rclpy
from typing import Union
from rdflib.plugins.sparql.processor import prepareQuery
from crow_control.utils import ParamClient
from rcl_interfaces.srv import GetParameters
from ros2param.api import call_get_parameters
from collections import deque
from npyscreen import TitleMultiSelect
from sensor_msgs.msg import CameraInfo
from threading import Thread
from datetime import datetime as dt
import numpy as np


npyscreen.CheckBox.True_box = '[o]'


class MainForm(npyscreen.TitleForm):
    MAX_CAMERA_DELAY = rclpy.time.Duration(seconds=0.5)
    CAMERA_MESSAGE_BUFFER_LEN = 10
    ALIVE_STAMP_BUFFER_LEN = 30
    FIX_MINIMUM_SIZE_WHEN_CREATED = False

    def create(self):
        self.min_l = 15
        self.node = self.parentApp.node

        self.nextrelx = 3
        self.add(npyscreen.Textfield, name='CAMERAS', value='CAMERAS', editable=False)
        self.nextrelx = 43
        self.nextrely -= 1
        self.add(npyscreen.Textfield, name='NODES', value='NODES', editable=False)
        self.nextrelx = 70
        self.nextrely -= 1
        self.add(npyscreen.Textfield, name='[last seen]', value='[last seen]', editable=False)
        self.nextrelx = 93
        self.nextrely -= 1
        self.add(npyscreen.Textfield, name='[alive signal delay stats]', value='[alive signal delay stats]', editable=False)
        self.nextrely += 1
        self.nextrelx = 1

        calib_client = self.node.create_client(GetParameters, '/calibrator/get_parameters')
        self.node.get_logger().info("Waiting for calibrator to setup cameras")
        calib_client.wait_for_service()
        # Retreive camera information
        cameras = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["camera_namespaces"]).values][0]
        while len(cameras) == 0: #wait for cams to come online
            self.node.get_logger().warn("No cams detected, waiting 2s.")
            time.sleep(2)
            cameras = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["camera_namespaces"]).values][0]

        qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.cameras = {}
        self.cam_name_list = {}
        self.cam_color_list = {}
        self.cam_depth_list = {}
        for cam in cameras:
            self.cam_name_list[cam] = self.add(npyscreen.Textfield, name=cam, value=cam, editable=False)
            tmp_nextrelx = self.nextrelx
            self.nextrelx = 10
            self.cam_color_list[cam] = self.add(npyscreen.CheckBox, name="/color", editable=False)
            self.cam_depth_list[cam] = self.add(npyscreen.CheckBox, name="/depth", editable=False)
            self.cameras[cam] = {
                'color': deque(maxlen=self.CAMERA_MESSAGE_BUFFER_LEN),
                'depth': deque(maxlen=self.CAMERA_MESSAGE_BUFFER_LEN),
            }
            self.nextrelx = tmp_nextrelx
            self.node.create_subscription(CameraInfo, cam + '/color/camera_info', callback=lambda ci_msg, cam=cam: self.update_camera_color(ci_msg, cam), qos_profile=qos)
            self.node.create_subscription(CameraInfo, cam + '/depth/camera_info', callback=lambda ci_msg, cam=cam: self.update_camera_depth(ci_msg, cam), qos_profile=qos)


        self.nextrelx = 40
        self.nextrely = 4
        self.pclient = self.parentApp.pclient
        self.alive_params = ["logic_alive", "detector_alive", "locator_alive", "filter_alive", "adder_alive", "nlp_alive"]
        self.alive_chb = {}
        self.alive_seen = {}
        self.alive_last = {}
        self.alive_stats = {}
        self.alive_stamps = {}
        for p in self.alive_params:
            self.pclient.declare(p, False)
            self.alive_chb[p] = self.add(npyscreen.CheckBox, name=p, editable=False)
            self.nextrelx += 30
            self.nextrely -= 1
            self.alive_seen[p] = self.add(npyscreen.Textfield, name=p + '_last', editable=False)
            self.nextrelx += 20
            self.nextrely -= 1
            self.alive_stats[p] = self.add(npyscreen.Textfield, name=p + '_stats', editable=False)
            self.nextrelx -= 50
            self.alive_last[p] = None
            self.alive_stamps[p] = deque(maxlen=self.ALIVE_STAMP_BUFFER_LEN)

        self.th = Thread(target=self.spin, daemon=True)
        self.th.start()
        self.node.create_timer(0.01, self.update)

    def spin(self):
        rclpy.spin(self.node)

    def beforeEditing(self):
        pass

    def afterEditing(self):
        self.parentApp.switchFormPrevious()

    def _count_fps(self, dq):
        if len(dq) > 1:
            total = []
            for i, q in enumerate(dq):
                if i == 0:
                    continue
                total.append((q - dq[i - 1]).nanoseconds)
            return 1 / ((sum(total) / len(total)) * 1e-9)
        else:
            return 0

    def _count_delay_stats(self, dq):
        if len(dq) > 1:
            total = []
            for i, q in enumerate(dq):
                if i == 0:
                    continue
                total.append(q - dq[i - 1])
            total = np.array(total)
            return total.mean(), total.min(), total.max(), total.std()
        else:
            return -1, -1, -1, -1

    def update(self):
        now = self.node.get_clock().now()
        for cam in self.cameras.keys():
            if len(self.cameras[cam]['color']) < 1 or len(self.cameras[cam]['depth']) < 1:
                continue
            last_color = self.cameras[cam]['color'][-1]
            last_depth = self.cameras[cam]['depth'][-1]
            self.cam_color_list[cam].value = now - last_color < self.MAX_CAMERA_DELAY
            self.cam_color_list[cam].name = f'/color [{self._count_fps(self.cameras[cam]["color"]):3.01f} fps]'
            self.cam_depth_list[cam].value = now - last_depth < self.MAX_CAMERA_DELAY
            self.cam_depth_list[cam].name = f'/depth [{self._count_fps(self.cameras[cam]["depth"]):3.01f} fps]'
            if self.cam_color_list[cam].value and self.cam_depth_list[cam].value:
                self.cam_name_list[cam].color = 'SAFE'
            else:
                self.cam_name_list[cam].color = 'DANGER'

        for p in self.alive_params:
            is_alive = getattr(self.pclient, p)
            self.alive_chb[p].value = is_alive > -1
            setattr(self.pclient, p, -1)  # reset the alive indicator
            if is_alive > -1:
                self.alive_last[p] = now
                when = "now"
                self.alive_chb[p].label_area.color = 'SAFE'
                self.alive_stamps[p].append(is_alive)
            else:
                diff = (now - self.alive_last[p]).nanoseconds * 1e-9
                if diff < 0.5:
                    self.alive_chb[p].label_area.color = 'STANDOUT'
                else:
                    self.alive_chb[p].label_area.color = 'DANGER'
                if self.alive_last[p] is None:
                    when = 'never'
                else:
                    when = f'{int(diff)} seconds ago'
            self.alive_seen[p].value = f'[{when}]'

            amean, amin, amax, astd = self._count_delay_stats(self.alive_stamps[p])
            if amean > 0:
                self.alive_stats[p].value = f'[mean: {amean:0.02f}\xB1{astd:0.02f}; [{amin:0.02f} - {amax:0.02f}]]'
                # print(self.alive_stamps[p])
                # print(amean, amin, amax, astd)
            else:
                self.alive_stats[p].value = f'[ N/A ]'

        self.display()

    def update_camera_color(self, ci_msg, camera):
        self.cameras[camera]['color'].append(rclpy.time.Time.from_msg(ci_msg.header.stamp))

    def update_camera_depth(self, ci_msg, camera):
        self.cameras[camera]['depth'].append(rclpy.time.Time.from_msg(ci_msg.header.stamp))


class Monitor(npyscreen.NPSAppManaged):

    def __init__(self, node):
        super().__init__()
        npyscreen.BufferPager.DEFAULT_MAXLEN = 500
        npyscreen.Popup.DEFAULT_COLUMNS = 100
        npyscreen.PopupWide.DEFAULT_LINES = 20
        npyscreen.PopupWide.SHOW_ATY = 1
        # initialize the ontology client
        self.node = node
        self.crowracle = CrowtologyClient(node=self.node)
        self.pclient = ParamClient()

        self.onto = self.crowracle.onto
        self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

    def onStart(self):
        # npyscreen.setTheme(npyscreen.Themes.TransparentThemeLightText)
        self.addForm("MAIN", MainForm, name="Monitor")
        return super().onStart()

    def onCleanExit(self):
        print("Exited cleanly")
        return super().onCleanExit()


def main():
    os.environ['ESCDELAY'] = "0.1"
    rclpy.init()
    node = Node("system_monitor_node")
    ot = Monitor(node)
    try:
        ot.run()
    except KeyboardInterrupt:
        ot.switchForm(None)
        print("User requested shutdown.")


if __name__ == '__main__':
    main()
