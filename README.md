# unitree-api-wrapper - for the Unitree Go1

## Setup

NOTE: REQUIRES Python 3.8 BC THAT'S WHAT THE UNITREE LIBRARY IS COMPILED FOR FOR SOME REASON

1. Copy or symlink [Go1's lib](https://github.com/unitreerobotics/unitree_legged_sdk/blob/master/lib/python/amd64/robot_interface.cpython-38-x86_64-linux-gnu.so) into the two following folders: `lib` and `unitree_api_wrapper/lib/`

2. Run `pip install -e .`

## Prepare the robot (Go1 only)

When you swap batteries and first boot up the robot, the following applies:

- Make sure you boot up the robot always in the same flat resting position, NOT on a rack/gantry. When you switch batteries, take it off the rack/gantry
- You will know that booting is done when the robot stands up.
- When the robot is standing up, it's running what's called "sport mode", which means it will listen to the gamepad for remote control and when you send commands they will clash with that. 
- So you need to terminate "sport mode". To do that run `./kill-sport-mode.sh` (password is `123`). If you've done that correctly, the robot will lie down. And then it's ready to be used with this library 

## Train policy

IMPORTANT:

This works best if you don't have to estimate the linear/angular velocity from IMU.
There are 2 tasks in https://github.com/simonchamorro/legged-gym-rl/ that support this:

`a1_flat_novel` and
`go1_flat_novel`

The difference between `go1_flat_novel` and `go1_flat` is that the former is trained without access to the ground truth linear and angular velocities of the robot body.


## Run policy

(Make sure you're using a jitted policy - see README over here: https://github.com/simonchamorro/legged-gym-rl/tree/main/legged_gym)

1. Copy your policy into the ./policies directory,
2. Then cd into the scripts folder `cd scripts`
3. Edit the file `05-run-base-policy.py` and replace the `policy_path` parameter with the name of your policy
4. Run the script `python 05-run-base-policy.py`
5. ...
6. Profit
