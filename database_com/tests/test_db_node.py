#!/usr/bin/env python
import sys
import unittest

import rospy
from database_com.msg import DBOperation
from database_com.srv import AddObject, AddObjectRequest, AddObjectResponse

PKG = 'test_roslaunch'

class TestDBNode(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node('test_node')

        self._addsrv = rospy.ServiceProxy('/db_interface_node/addObject', AddObject)

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
        self.assertIs(res.res.success, True, 'Adding Object Service responded with False')



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_db_node', TestDBNode)