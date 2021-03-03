##PLANNER - BUILD DOG
#plan(suggestion = "user_input_argument")
"""
1) choose suggested/first object in assembly - it defines assembly's reference frame
2) find connections that involve this object and are not done yet
3) do all found connections (connection can be done if connstraint are not violated (relation, order, object.parts enabled))
    - try another connection if this one is not possible to do
        - move to another object (that is already involved in assembly, not a random one) if no connections can be done, repeat 2,3 (get back to this one later)
4) move on to another object (that is already involved in assembly, not a random one)
5) repeat 2,3,4 until all connections are done
    - start over from 1 if plan can't be finished
        - return error if plan can't be finished from any object in assembly

legend:
? ask ontology
+ add to ontology
# implemented funcion
((=> returned valu))
## comment...
"""
import numpy as np

id_start = 0
#start(suggestion = {start_with = cube}):
    plan = {}
    objects = [] ##objects added to assembly that have connections to do
    ? objects_list = is_in_assembly(DOG, Object, None)) ((=> [all objects in DOG]))
    ? connections_list = is_in_assembly(DOG, Connection, None)) ((=> [all connections in DOG]))
	? object0_list = is_in_assembly(DOG, Object, cube) ((=> [CUBE_FRONT, CUBE_BACK]))
	if object0_list = None: 
		? object0_list = objects_list ((=> [all objects in DOG]))
	try:
        object0 = object0_list[id_start] ((=> CUBE_FRONT))
    except:
        print("There is no valid plan for this assembly")
        return -1
	ref_frame = [[0,0,0],[1,0,0],[0,1,0],[0,0,1]] ##realtime: vision+?+calculate => [[object0.AbsoluteLocation],[object0.xvec],[object0.yvec],[object0.zvec]] => transformation
	+ CUBE_FRONT.AbsoluteLocation = [0,0,0], 
    + CUBE_FRONT.hole1.AbsoluteLocation = CUBE_FRONT.AbsoluteLocation + CUBE_FRONT.hole1.RelativeLocation, ..., 
    + CUBE_FRONT.hole1.vector = CUBE_FRONT.AbsoluteLocation-CUBE_FRONT.hole1.AbsoluteLocation, ...
    objects.append(object0) ##add object to objects added to assembly that have connections to do

id_object = 0
while len(connections_list) > 0: ##some connections not done yet
    try:
        #find_connection(object0 = objects[id_object]):	
            ? object_connection_list = is_in_connection(CUBE_FRONT,None) ((=> [[NECK1, PEG_NECK], [BODY1, PEG_BODY], [FRONT1, SCREW_FRONT]]))
            object_connection_list = [x for x in object_connection_list if x.vector is None] ##connections not done yet
            id_conn = 0
            while len(object_connection_list) > 0: ##some connections not done yet
                object = object_connection_list[id_conn][1] ((=> PEG_NECK))
                conn = object_connection_list[id_conn][0] ((=> NECK1))

                #check_connection(conn):
                    ? order_list = is_in_order(NECK1,"then") ((=> None))
                    for order in order_list:
                        if order.type == "RequiredOrder" and order.first.vector is None: ##required preceeding connection not done yet
                            return False
                    return True

                if check_connection(conn):
                    #do_connection(object = object0, object = object, connection = conn): 
                        conn_type = ? NECK1.type
                        conn_relation_list = ? is_in_relation(NECK1,None) ((=> BODY1, FRONT2, BACK2))
                        
                        #check_connection_actors(object=CUBE_FRONT, object=PEG_NECK):
                            ## based on conn_type:
                            ? hole_list = is_enabled(CUBE_FRONT,)
                            vector_list = [hole.vector for hole in hole_list] ((=> [[0,0,-a/2],[0,0,a/2],...]))
                            
                            #get_possible_vector(conn_relation_list, vector_list): 
                                for relation in conn_relation_list:
                                    if relation.vector is not None:
                                        ##based on relation.type:
                                        vector_list = get_perpendicular(relation.vector, vector_list)) ((=> full vector_list)) ##no constraints apply
                            
                            if (len(vector_list) != 0) and
                                ((PEG_NECK.shank.used += NECK1.length) <= PEG_NECK.shank.length) and
                                (PEG_NECK.thread1.enabled) and
                                return True
                            else:
                                return False
                        
                        if check_connection_actors(object0, object): ##success
                            + NECK1.vector = vector_list[0] ((=> [0,0,-a/2]))
                            + CUBE_FRONT.hole1.enabled = False
                            + PEG_NECK.thread1.enabled = False
                            + PEG_NECK.shank.used += NECK1.length ((=> 15))
                            + PEG_NECK.AbsoluteLocation = ...
                            + PEG_NECK.thread1.AbsoluteLocation = ...
                            + PEG_NECK.thread2.AbsoluteLocation = ...
                        
                            objects.append(object) ##add object to objects added to assembly that have connections to do
                            plan_step_id = connection_list.pop(conn) ##reduce list of not done connections
                            plan[str(plan_step_id)] = conn ##add connection to the plan
                            object_connection_list.pop(conn) ##reduce list of not done connections of this object
                            id_conn = 0 ##start this loop again with reduced object_connection_list
                        else: ##failure
                            id_conn += 1 ##continue with next connection in object_connection_list
                
                if "all possible combinations of connections for this object tried ":
                    id_object +=1
                    break ##stop trying with this object and go on with next already assembled object
                    ##if there are no more objects to try, go to this while's "except"
            
            objects.pop(object) ##remove object if all its connections are done
            id_object = 0 ##continue with next object in objects added to assembly
    except:
        print("Plan starting with object0 impossible")
        id_start += 1 ##try to start with another object
        #start(suggestion)

print("DONE, plan:", plan) ##plan is reconstructed from ids of connections that were popped from connection_list
return plan

##helper funtions
def is_perpendicular(vector1, vector2):
    """
    Parameters:
        :param vector1: (list of arrays or array) Existing connections
        :param vector2: (array) New connection to check
    """
    threshold = 0.1
    for vector in vector1:
        dot_product = (np.dot(vector, vector2)) / (np.linalg.norm(vector) * np.linalg.norm(vector_2))
        angle = np.arccos(np.dot)
        if ((np.pi/2 - threshold) <= angle) or (angle >= (np.pi/2 + threshold)):
            return False
    return True
	
def get_perpendicular(vector1, vector2):
    """
    Parameters:
        :param vector1: (list of arrays or array) Existing connections
        :param vector2: (list of arrays or array) Possible new connections to check
    """
    perp = []
    for vector in vector2:
        if is_perpendicular(vector1, vector):
            perp.append(vector)
    return perp

def is_opposite(vector1, vector2):
    """
    Parameters:
        :param vector1: (list of arrays or array) Existing connections
        :param vector2: (array) New connection to check
    """
    threshold = 0.1
    for vector in vector1:
        dot_product = (np.dot(vector, vector2)) / (np.linalg.norm(vector) * np.linalg.norm(vector_2))
        angle = np.arccos(np.dot)
        if ((np.pi - threshold) <= angle) or (angle >= (0 + threshold)):
            return False
    return True
	
def get_opposite(vector1, vector2):
    """
    Parameters:
        :param vector1: (list of arrays or array) Existing connections
        :param vector2: (list of arrays or array) Possible new connections to check
    """
    opp = []
    for vector in vector2:
        if is_opposite(vector1, vector):
            opp.append(vector)
    return opp
