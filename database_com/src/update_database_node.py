from typing import ClassVar

import rospy
from geometry_msgs.msg import PoseStamped
from object_detection_aruco.msg import MarkersAvgList, MarkersAvg
from database_com.msg import DatabaseUpdate
import logging

# necessary to load the ontology classes even if seems unused
from nlp_crow.database.Ontology import *
from nlp_crow.database.Database import Database


import owlready2 as ow

db = Database()

with db.onto as onto:
    class update_object_position_node():
        # listens to /averaged_markers from object_detection package and in parallel to button presses. When there is an approval
        # to collect cubes (key a -> y), robot picks up the currently detected cubes and places them to storage
        def __init__(self):
            rospy.init_node('update_object_position_node', anonymous=False)

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

          #  rospy.on_shutdown(self.shutdown_hook)


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
                    obj = onto.search(type=onto.Cube)
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
                        self.add_object(onto.Cube, x=pose.position.x, y=pose.position.y, id=id)
                        print(onto.search(type=onto.Cube, _is_in_workspace=True))
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
                color = onto.NamedColor(color)
                obj.color.append(color)

            point = onto.Point2D(x=x, y=y)
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