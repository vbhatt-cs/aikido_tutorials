# Aikido Tutorials

## Installation (With singularity)

1. Install [singularity](https://docs.sylabs.io/guides/3.6/user-guide/quick_start.html#quick-installation-steps).
1. Create the container: `sudo singularity build container.sif container.def`. The container will have all the pre-requisites including ROS and AIKIDO.
1. Create a singularity shell: `SINGULARITYENV_DISPLAY=$DISPLAY singularity shell --cleanenv container.sif`. `$DISPLAY` environment variable is passed to allow rviz to work correctly both on a local computer and when using X11 through SSH. The following commands can also be run through `singularity run ...` or `singularity exec ...` but it is easier to create a singularity shell and execute the commands inside.

Run the following commands inside the singularity shell:

1. Create a catkin workspace in the current directory: `catkin init`
1. Build the `aikido_tutorials` package: `catkin build`
1. Source setup file: `. devel/setup.bash`

## Installation (Without singularity)

1. Install all the apt packages in the `%post` section of `container.def`.
1. Create a catkin workspace and install the dependencies:
   
```
# Catkin dependencies
cd src
wstool init
wstool merge ../ros_requirements.rosinstall
wstool up

# Build catkin workspace
cd ..
catkin init
catkin config --extend /opt/ros/noetic
catkin build
```

3. Source setup file: `. devel/setup.bash`

## Running

1. Run `roscore`, `rosrun rviz rviz` and `rosrun aikido_tutorials aikido_tutorials` in three separate shells (singularity shells if using singularity).
1. In the rviz window, add `InteractiveMarkers` with update topic `/dart_markers/aikido_tutorial/update`
1. Pressing "Y" when prompted in `rosrun aikido_tutorials aikido_tutorials` shell should make the robot move between two configurations. 
