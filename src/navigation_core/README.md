# Simulation for harwey
Simulation environment for the robot. URDFs and SDFs and configs for the simulation

## Commands for starting simulation

# Without launch files

1. Launch the simulation environment from ros2, standing in ros2_harvey root folder:
    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=src/navigation_core/worlds/slu_simulation.sdf 
2. To launch the robot description. In new terminal write:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro simulation/robot_models/robot.urdf.xacro)"
3. Spawn robot:
    ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=src/navigation_core/worlds/slu_simulation.sdf topic:=robot_description entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5