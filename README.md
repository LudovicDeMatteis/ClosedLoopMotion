# ClosedLoopMotion
Examples of motion of robot with closed kinematic loops.
The repo is based on methods described in ICRA 2025 paper - in review (TODO add paper reference).

## Dependencies
To build this projects, the following are required :
* [Crocoddyl fork](https://github.com/LudovicDeMatteis/crocoddyl/tree/topic/contact-6D-closed-loop) on branch `topic/contact-6D-closed-loop`
* [sobec](https://github.com/LudovicDeMatteis/sobec/tree/save-icra) on branch `save-icra`
* [toolbox-parallel-robots](https://github.com/Gepetto/toolbox-parallel-robots)
* [example-parallel-robots](https://github.com/Gepetto/example-parallel-robots)

We will provide a direct installation as soon as possible

## Table of contents 
* The repo contains utility loaders based on [example-parallel-robots](https://github.com/Gepetto/example-parallel-robots), included in the `loaders` folder. Current version supports Battobot and Digit. The models of Disney robot and Kangaroo are to be fixed and added (cf [Future Improvements](#future-improvements))
* Params for different motions on included in folder `params`.
* The folder `tests` contains tests on crocoddyl functions.
* Other scripts are examples of motions on different robots, named `{motion}_{robot}_{open/closed}.py`

## Usage
Running script `main.py` will open a GUI to select the desired motion, the desired robot and allow setting some motion parameters.

## Future Improvements
We list in arbitrary order the expected improvements
* Adding new robot models (Disney robot, Kangaroo...)
* Adding new motions (half-turn, flip, running...)