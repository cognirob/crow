#!/usr/bin/env python
import pickle
import sys
import unittest

import rospy
from database_com.msg import DBOperation
from database_com.srv import AddObject, AddObjectRequest, AddObjectResponse, GroundProgram, GroundProgramRequest
from database_com.srv import GetDatabase, GetDatabaseRequest, GetDatabaseResponse

from nlp_crow.database.Database import Database
from nlp_crow.processing.NLProcessor import NLProcessor

PKG = 'test_roslaunch'

class TestDBNode(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node('test_node')

        self._addsrv = rospy.ServiceProxy('/db_interface_node/addObject', AddObject)
        #self._groundsrv = rospy.ServiceProxy('/db_interface_node/groundProgram', GroundProgram)
        self._getdatsrv = rospy.ServiceProxy('/db_interface_node/getDatabase', GetDatabase)

    def test_add_object(self):
        self.assertEqual(1,1, msg='1!=1')
        req = AddObjectRequest()
        req.x = 13
        req.y = 2
        req.id = '25'
        req.obj_class = 'Cube'
        req.color = 'red'

        req.action.action = DBOperation.ADD


        res = self._addsrv.call(req)
        self.assertIsInstance(res, AddObjectResponse)
        print(res)

        self.assertIs(res.res.success, True, 'Adding Object Service responded with False')

    # def test_ground_program(self):
    #     req = AddObjectRequest()
    #     req.x = 1
    #     req.y = 3
    #     req.id = '2'
    #     req.obj_class = 'Cube'
    #     req.color = 'red'
    #     res = self._addsrv.call(req)
    #     print(res)
    #     nl_processor = NLProcessor()
    #     input_sentence = 'Pick a red cube'
    #     program_template = nl_processor.process_text(input_sentence)
    #     req = GroundProgramRequest()
    #     req.ungroundProgram = pickle.dumps(program_template, pickle.HIGHEST_PROTOCOL)
    #     res = self._groundsrv.call(req)
    #     gr_prog = pickle.loads(res.groundedProgram)
    #     print(gr_prog)

    def test_get_database(self):
        req = GetDatabaseRequest()
        req.write = True
        db = Database()
        obj = db.onto.search(id = '25')
        print(obj)
        res = self._getdatsrv.call(req)

        db.change_onto(res.path)
        obj = db.onto.search(id = '25')
        print(obj)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_db_node', TestDBNode)