#!/usr/bin/env python
"""
Copyright (c) 2019 Karla Stepanova, Zdenek Kasner
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova, Zdenek Kasner
"""
import os
from typing import ClassVar

import rospy
# from database_com.srv import GroundProgram, GroundProgramResponse, GroundProgramRequest
from database_com.srv import AddObject, AddObjectResponse, AddObjectRequest
from database_com.srv import GetDatabase, GetDatabaseResponse, GetDatabaseRequest

import pickle

from geometry_msgs.msg import PoseStamped
from object_detection_aruco.msg import MarkersAvgList, MarkersAvg
from database_com.msg import DatabaseUpdate
import logging

# necessary to load the ontology classes even if seems unused
from nlp_crow.database.Ontology import *
from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI


import owlready2 as ow
import sys

from nlp_crow.processing.ProgramRunner import ProgramRunner

db = Database()



#with db.onto as onto:
with db.onto:
    class update_object_position_node():
        # listens to /averaged_markers from object_detection package and in parallel to button presses. When there is an approval
        # to collect cubes (key a -> y), robot picks up the currently detected cubes and places them to storage
        def __init__(self):
            rospy.init_node('db_interface_node', anonymous=False)

            self._dbapi = DatabaseAPI()

            # global tf_buffer
            # tf_buffer = tf2_ros.Buffer(rospy.Duration(50.0))  # tf buffer length
            # global tf_listener
            # tf_listener = tf2_ros.TransformListener(tf_buffer)
            # rospy.sleep(1.0)
            #
            # rospy.Subscriber("/stable_disc_markers", MarkersAvgList, self.cubes_callback)
            self.sub_cubes = rospy.Subscriber('/averaged_markers', MarkersAvgList, self.update_object_position)
            # self.listener = keyboard.Listener(on_press=self.on_press)
            # self.listener.start()
            self.pub = rospy.Publisher('/database_update', DatabaseUpdate, queue_size=10)

            self.srv = {}
            # self.srv['groundProgram'] = rospy.Service('~groundProgram', GroundProgram, self.handle_ground_program)

            self.srv['addObject'] = rospy.Service('~addObject', AddObject, self.handle_add_object)
            self.srv['getDatabase'] = rospy.Service('~getDatabase', GetDatabase, self.handle_get_database_request)



          #  rospy.on_shutdown(self.shutdown_hook)

        def handle_add_object(self, req):
            """
            Updates the position of the existing object or adds the object if it does not exist yet.

            Parameters
            ----------
            id  the identifier of the object, e.g. ArUco id
            x   the position of the object: x coordinate
            y   the position of the object: y coordinate
            """
            assert isinstance(req, AddObjectRequest)
            res = AddObjectResponse()
            obj = db.onto.search_one(id=req.id, _is_in_workspace=True)
            choices = {'Cube': db.onto.Cube, 'Panel': db.onto.Panel,'Glue': db.onto.Glue}
            typeO = choices.get(req.obj_class, 'Cube')

            if obj is None:
                self._dbapi.add_object(Class=typeO, x=req.x, y=req.y, id=req.id, color=req.color)
                res.res.msg = 'I added a {} {} with id {} at x = {} and y = {}'.format(req.color, req.obj_class, req.id, req.x, req.y)
                res.res.success = True
            else: #TODO if change of position > threshold update position
                obj.location.x = req.x
                obj.location.y = req.y
                res.res.msg = 'I updated the position of a {} {} with id {} from x = {}, y = {} to x = {} and y = {}'.format(req.color, req.obj_class, req.id, obj.location.x, obj.location.y, req.x, req.y)
                res.res.success = True

            #TODO update other parameters if changed...
            # TODO test whether the object arrived in the database
            obj = db.onto.search(id=req.id, _is_in_workspace=True)
            print(obj)

            return res



        # def handle_ground_program(self, req):
        #     # type: (GroundProgramRequest) -> GroundProgramResponse
        #     res = GroundProgramResponse()
        #     u_prog = pickle.loads(req.ungroundProgram)
        #     assert isinstance(u_prog, db.onto.RobotProgram)
        #
        #     program_runner = ProgramRunner()
        #     robot_program = program_runner.evaluate(u_prog)
        #     res.groundedProgram = pickle.dumps(robot_program, pickle.HIGHEST_PROTOCOL)
        #     print('Robot program grounded:')
        #     print(robot_program)
        #
        #
        #     return res

        def handle_get_database_request(self, req):
            # type: (GetDatabaseRequest) -> GetDatabaseResponse
            path = os.path.dirname(os.path.abspath(__file__)) + '/saved_onto.owl'
            db.onto.save(path)
            if req.write:
                #lock database
                print('write')
            else:
                #do not lock and continue happily writing to it
                print('read')
            res = GetDatabaseResponse()
            res.path = path

            return res


        def update_object_position(self, data):
        #def update_object_position(self, id: str, x: float, y: float):
            """
            Updates the position of the existing object or adds the object if it does not exist yet.

            Parameters
            ----------
            id  the identifier of the object, e.g. ArUco id
            x   the position of the object: x coordinate
            y   the position of the object: y coordinate
            """
            if len(data.markers)>0:
                # if pub is None:
                #     pub = rospy.Publisher('/database_update', DatabaseUpdate, queue_size=10)

                msg = DatabaseUpdate()
                msg.header = data.header  # pose_transformed_obj.header
                msg.id = []
                msg.update_id = []

                for i in range(0, len(data.markers)):
                    pose = data.markers[i].avg_pose
                    id = data.markers[i].id

                    #obj = onto.search(aruco_id=1,_is_in_workspace=True)
                    obj = db.onto.search(type=db.onto.Cube)
                    addObj = True
                    for i in range(0,len(obj)):
                        if (obj[i].aruco_id == id) and (obj[i]._is_in_workspace == True):
                            addObj = False
                            idx = i
                    #obj = True
                    changed_position = True

                    if addObj:
                        # creates a new cube
                        # TODO make it work with any object
                        self.add_object(db.onto.Cube, x=pose.position.x, y=pose.position.y, id=id)
                        print(db.onto.search(type=db.onto.Cube, _is_in_workspace=True))
                        msg.id.append(id)
                        msg.update_id.append(1)
                    elif changed_position and not addObj :
                        obj[idx].location.x = pose.position.x
                        obj[idx].location.y = pose.position.y
                        msg.id.append(id)
                        msg.update_id.append(2)  # should be 2 only if pose actually updated
                self.pub.publish(msg)

                if len(data.markers) == 0:
                    return

        def add_object(self, Class: ClassVar, x : float, y : float, id : str = None, color : str = None):
            """
            Adds a new object in the ontology.

            Parameters
            ----------
            Class the class of the ontology object to be added, sho Class: ClassVar, x : float, y : float, id : str = None, color : str = Noneuld be a subclass of onto.Object
            x     the position of the object: x coordinate
            y     the position of the object: y coordinate
            id    the identifier of the object, e.g. ArUco id
            color the color of the object as string, e.g. "red"
            -------

            """
            obj = Class()

            if id:
                obj.aruco_id = id

            if color:
                color = db.onto.NamedColor(color)
                obj.color.append(color)

            point = db.onto.Point2D(x=x, y=y)
            obj.location = point

            # TODO without this, some objects may be involuntarily destroyed by a garbage collector
            # TODO this should be investigated further to avoid some magical errors
            db.objects.append(obj)

            # set the internal attribute which signifies this is a real object in the workspace
            obj._is_in_workspace = True

            # obj._can_be_picked = True

    if __name__ == '__main__':
        update_object_position_node()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()