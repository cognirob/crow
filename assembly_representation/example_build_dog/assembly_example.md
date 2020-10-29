## Introduction
The goal is to find a way to describe assembly operations with crow objects ([wiki](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/wikis/crow-objects)). There are several complex objects designed ([wiki](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/wikis/uploads/137bc0bbecb1544aa61fbb1c1e848329/stavebnice_build_objects.pdf)), their step-by-step assembly is represented in a form of a tree ([wiki](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/wikis/uploads/5829ddaa9acbf73fcd06579325948d33/stavebnice_strom.pdf)).

### Locations
The [original template](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/old_json/full_template_SC2_u.json) was extended. The key problem is how to define locations, since no xyz coordinates are known in advance. New keywords are used ([description.yaml](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/description.yaml)) to describe locations. Locations are in a form of functions (**floc**) and are evaluated using vision + some computing (rigid transformations, e.g. between cube's center and the center of a hole in its side). A floc *object("cube_holes")* asks vision to return centroids of all cubes visible at the moment when it is called (it is a list of unknown length (prior to evaluation)). Objects may have parts (*hole, end*) and these have properties (*empty, full*). There is some module that keeps track of these properties and of changing locations of identified objects/parts as the assembly proceeds. 

Some locations need to be defined w.r.t. other objects/parts using properties (*opposite, perpendicular*). Rather than using *opposite to cube* we define it using locations previously visited by an **ovc**. The function *from(ovc_xy)* points to the locations visited by ovc_xy (if there are multiple, it returns a list). The query can be specified by *before, after, now*. *before* returns the location of an object/part before ovc_xy visited it, *after* returns the location where it moved right after that, *now* returns the location at the moment when *from(ovc_xy)* is called.

Set operations with floc (*union, difference*) are defined.

### Planning
During planning, planner interacts with vision and tracking/computing module, each floc is evaluated just in time.

### Actions and hierarchy
The assembly representation for each complex object from the [assembly tree](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/wikis/uploads/5829ddaa9acbf73fcd06579325948d33/stavebnice_strom.pdf) is in separate yaml file. 

There are basic actions *pick'n'place, pick'n'hold* and actions representing usage of a tool *hammer, wrench, pliers, screwdriver*. Some typical operations are called an action (*attach_a, detach_d*). Assembly of each complex object as *build_0a1a* can become an action too. The assembly of a complex object is always based on action *build an object one level less complex* and *do necessary operations to make it one level more complex*.

## Example build_dog:
[Dog](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/build_dog.yaml) is a level-3-assembly. There is preceding 3rd level complex object [0a1a2a3a](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/build_0a1a2a3a.yaml), preceding 2nd level complex object [0a1a2a](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/build_0a1a2a.yaml) and 1st level complex object [0a1a](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/build_0a1a.yaml) consisting of 0th (crow) object cube_holes.

![Build_dog action diagram](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/example_build_dog/build_dog_action_diagram.jpeg)

![Build_dog flow](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/example_build_dog/build_dog_flow.jpeg)

### Illustrated description
![Build the dog](https://gitlab.ciirc.cvut.cz/imitrob/project_crow/crow/-/blob/master/assembly_representation/example_build_dog/build.jpeg)

### Natural language description
- First build 0a1a2a3a (body)
  - First build 0a1a2a (head and belly)
    - First build 0a1a
      - First take cube_holes 0a and put it on the target place
      - Then attach c to any empty hole in the cube
        - Put a peg into the target hole and hold it there
        - Hammer the peg into the hole
    - Then take a sphere_holes and put the free end of the peg from 0a1a into any empty hole in the sphere. Hold it there.
    - Hammer the sphere onto the peg
  - And build 0a1a (bottom)
    - First take cube_holes 0a and put it on the target place
    - Then attach c to any empty hole in the cube
      - Put a peg into the target hole and hold it there
      - Hammer the peg into the hole
  - Put the free end of the peg from 0a1a (bottom) into an empty hole in the cube from 0a1a2a (belly) so, that the peg from 0a1a is in any hole perpendicular to the full hole. Hold it there.
  - Hammer the two complexes together (force the peg into the hole).
- Attach b (front legs) to any empty hole in the cube from 0a1a2a (belly), that is perpendicular to both full holes in that cube
  - Take a wafer and put its empty hole onto the target hole
  - Take a screw and put its thread through the wafer hole and target hole
  - Take another wafer and put its empty hole on the thread of the screw
  - Then attach d on the thread of the screw
    - Take a nut and put it on the target thread
    - Take a screwdriver and put it on the screw head, hold it
    - Take a wrench and put either of its ends on the nut, screw the nut to fix it
- Attach b (back legs) to an empty hole in the cube from 0a1a (bottom), that is perpendicular to a full hole in that cube and also perpendicular to the peg from 0a1a2a
  - Take a wafer and put its empty hole onto the target hole
  - Take a screw and put its thread through the wafer hole and target hole
  - Take another wafer and put its empty hole on the thread of the screw
  - Then attach d on the thread of the screw
    - Take a nut and put it on the target thread
    - Take a screwdriver and put it on the screw head, hold it
    - Take a wrench and put either of its ends on the nut, screw the nut to fix it
- Attach c (tail) to the empty hole in the cube in 0a1a (bottom) that is opposite to the full hole where the peg is
  - Put a peg into the target hole and hold it there
  - Hammer the peg into the hole
- Attach c (nose) to an empty hole in the sphere (head) that is perpendicular to the one full with the peg
  - Put a peg into the target hole and hold it there
  - Hammer the peg into the hole

### A possible planning scenario (start with build_0a1a)
- -> floc:cubes? (vision) => [loc1, loc2, ..., locNholes]
  - Choose loc (loc1[0]) 
  - Put it to loc (loc1[1])
- -> floc:empty_holes? (computing) => [loc1, loc2, ..., loc6]
  - Choose loc (loc2[0])
  - -> floc:pegs? (vision) => [loc1, loc2, ..., locNpegs]
  - Choose loc (loc2[1])
  - Put loc2[1] into loc2[0], hold
- -> floc:hammer? (vision) => [loc1, loc2, ..., locNhammers]
  - Choose loc (loc3[0])
  - Hammer at loc2[1].now (computing) => [loc1] (loc3[1])
- ...  