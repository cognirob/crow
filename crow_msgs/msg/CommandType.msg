int64 NOOP=0
# 1, 2, 4, 8 is used as flags for other commands
int64 GRIP=16  # close the gripper
int64 RELEASE=32  # open the gripper

int64 POINT=64  # point towards a location
int64 PICK=80  # pick an object
int64 PLACE=128  # place an object (assuming something is being held)
int64 PNP=208  # pick n place
int64 STORE=209  # pick an object an place it into a storage
int64 STASH=210  # pick an object an place it into the robot storage (backstage)
int64 FETCH=212  # take object from backstage and put it near the user
int64 FETCH_TO=213  # take object from backstage and put it into a storage

int64 STOP=256  # stop the robot arm(s)
int64 RETRACT=512  # retract to starting position
int64 MOVE=2048  # move eef to a location

int64 REM_CMD_LAST=4294967296  # removes last added command
int64 REM_CMD_X=4294967297  # removes the specified command

int64 DEFINE_STORAGE=8589934592 # start process of marker recognition, polyhedron calculation and addition of this entry to the ontology