# Can we use depth cameras as tactile sensors for quadruped locomotion?



## Reference

This work is built on top of Rex-Gym:   https://github.com/nicrusso7/rex-gym 



## What code is written by me?

Added `render_necek` function  and `render_foot` function in  `rex_gym/envs/rex_gym_env.py` to add depth cameras to mimic tactile sensor.

Created a new robot  `rex_gym/util/pybullet_data/robot_tactile.urdf` which has shell on feet to hold cameras

Modified `rex_gym/envs/gym/walk_env.py` to feed depth image observation to the training pipeline.

Modified the global parameters in `rex_gym/model/terrain.py` to generate different terrains for experiments.

Modifed the network structure in `rex_gym/agents/scripts/networks.py` by adding new observation layers.

Modified `rex_gym/playground/policy_player.py` so I note the status of robot.

Created  `main.py` so I don't need to install the package.

## Beyond writing code

Most importantly, to make these modifications and to make them work, I read and understood ALL code in this repository and learned how to use RNN in tensorflow(which is much unfridenly to beginners than pytorch in my opinion).
