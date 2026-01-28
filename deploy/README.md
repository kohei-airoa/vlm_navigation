# How to Deploy pi0

## 1. Installation
```bash
# Clone the main repository
git clone https://git.hsr.io/hsrtx/hsr_openpi.git

# Switch to the deployment branch
git switch feature/deploy
# clone submodules
git clone --recurse-submodules git@github.com:Physical-Intelligence/openpi.git
```

## 2. Environment Setup

Set the following environment variables

```bash
# Project configuration
export DEEP_PROJECT_NAME="your_project_name"    # Name of your project
export ROBOT_NAME="hsr_name"                    # Robot name (ex: hsrb107)
export ROS_IP="your_ip_address"                 # Your machine's IP address
```

## 1. Build the Docker Image
```bash
./BUILD-DOCKER-CONTAINER.sh　deploy
```

## 2. Place the model weights into a directory (e.g., checkpoints)

## 3. Launch the Docker Container
```bash
./RUN-DOCKER-CONTAINER.sh deploy
```

## 4. Install Dependencies

We use [uv](https://docs.astral.sh/uv/) to manage Python dependencies. See the [uv installation instructions](https://docs.astral.sh/uv/getting-started/installation/) to set it up. Once uv is installed, run the following to set up the environment:

```bash
cd /home/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync
```

## 4. Inside the container, edit the following file to resolve the stack issue in rospy's init:
Note: This step might not be necessary for ROS Noetic versions after May 25, 2022. You can try skipping this section if you're using a recent version.

/opt/ros/noetic/lib/python3/dist-packages/rosgraph/roslogging.py

Change this block:
modify findCaller funciton in LoggingException class
```python
while hasattr(f, "f_code"):
    # Search for the right frame using the data already found by parent class.
    co = f.f_code
    filename = os.path.normcase(co.co_filename)
    if filename == file_name and f.f_lineno == lineno and co.co_name == func_name:
        break
    if f.f_back:
        f = f.f_back
```
To this (just add one break):
```python
while hasattr(f, "f_code"):
    # Search for the right frame using the data already found by parent class.
    co = f.f_code
    filename = os.path.normcase(co.co_filename)
    if filename == file_name and f.f_lineno == lineno and co.co_name == func_name:
        break
    if f.f_back:
        f = f.f_back
        break
```
Reference: https://github.com/ros/ros_comm/issues/2296

## 5. Build Ros package
    ```bash
    ./RUN-DOCKER-CONTAINER.sh
    roscd
    catkin build
    source devel/setup.bash
    ```

## 6. Run the model inference
Before run the script. please check the following files:
- src/openpi/training/config.py: please ensure the configuration is set correctly for your model and dataset.
- deploy/hsr_data_collection/hsr_data_collection/config/hsr_data_collection_config.yaml : ensure the configuration is set correctly for your controller. (ps3 or ps4)

Firstly, connect a DualShock 3 or 4 controller to the HSR.
```bash
roslaunch hsr_openpi hsr_openpi.launch \
	config_name:=pi0_hsr_weblab_leader \
	checkpoint_dir:=/home/openpi/checkpoints/pi0_hsr_weblab_leader/my_experiment/99
```
To change the language instruction at runtime:
```bash
rosservice call /hsr_openpi/update_instruction "message: 'Open the oven toaster'"
```

## 7. Start model action execution
Press the left directional button of controller to start the action.