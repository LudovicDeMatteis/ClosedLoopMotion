# ClosedLoopMotion
Examples of motion of robot with closed kinematic loops.
This repo illustrates methods proposed in [ICRA 2025 submission](https://gepettoweb.laas.fr/articles/ldematte_icra2025.html)

## How to cite
```bibtex
@unpublished{dematteis:hal-04716938,
  TITLE = {{Optimal Control of Walkers with Parallel Actuation}},
  AUTHOR = {de Matte{\"i}s, Ludovic and Batto, Virgile and Carpentier, Justin and Mansard, Nicolas},
  URL = {https://hal.science/hal-04716938},
  NOTE = {working paper or preprint},
  YEAR = {2024},
  MONTH = Oct,
  PDF = {https://hal.science/hal-04716938v1/file/ICRA_2025___Optimal_Control_of_Closed_Loop_walkers-submission.pdf},
  HAL_ID = {hal-04716938},
  HAL_VERSION = {v1},
}
```

## Installation
### Using docker
- Pull the docker image
```
docker pull gitlab.laas.fr:4567/ldematteis/nix-closedloopmotion
```
- Run the docker image 
```
docker run --rm --network host -it gitlab.laas.fr:4567/ldematteis/nix-closedloopmotion:latest
```
- Inside the docker
```
git clone https://github.com/LudovicDeMatteis/ClosedLoopMotion
cd ClosedLoopMotion/
```
### From source
To use this project, following dependencies are required:
* [Crocoddyl fork](https://github.com/LudovicDeMatteis/crocoddyl/tree/topic/contact-6D-closed-loop) on branch `topic/contact-6D-closed-loop`
* [sobec](https://github.com/LudovicDeMatteis/sobec/tree/icra-2025) on branch `icra_2025`
* [toolbox-parallel-robots](https://github.com/Gepetto/toolbox-parallel-robots)
* [example-parallel-robots](https://github.com/Gepetto/example-parallel-robots)

## Usage
Run either
```
python main.py
```
or 
```
python main_nogui.py
```
You can then choose the desired motion, the robot and set parameters.
Once the solver ends, you can replay the final trajectory or show plots of the results.
Make sure to open the [Meshcat](https://github.com/meshcat-dev/meshcat-python) visualizer at  `http://127.0.0.1:7000/static/`

## Table of contents 
* `loaders` - The repo contains utility loaders based on [example-parallel-robots](https://github.com/Gepetto/example-parallel-robots), included in the `loaders` folder. Current version supports Battobot and Digit. The models of Disney robot and Kangaroo are to be fixed and added (cf [Future Improvements](#future-improvements))
* `params` - Params for different motions on included in folder `params`.
* `tests` - This folder contains tests on crocoddyl functions, this should be complete.
* `motions` - These scripts are examples of motions on different robots, named `{motion}_{robot}_{open/closed}.py`
* `main.py` creates a GUI using tkinter and asks for a desired motion, desired robot and parameters options.
* `main_nogui.py` does the same as `main.py` with a text interface (to allow use in docker) 

## Future Improvements
We list in arbitrary order the expected improvements
* Adding new robot models (Disney robot, Kangaroo...)
* Adding new motions (half-turn, flip, running...)
* Add tests for crocoddyl functions