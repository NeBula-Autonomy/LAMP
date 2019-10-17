# Tools for LAMP

This module contains scripts to use when running LAMP.

Always source your `workspace/devel/setup.bash` (or `internal/devel_isolated/setup.bash`) in .bashrc before running the loop closure tools. The `blam_slam` ROS node must be running while executing any of the following services.

This package requires the Python package `Transforms3d`, which can be installed via

```sh
pip install transforms3d
```

## How to run the tools

The tools can be run through rqt with messages. Recommended is to run rqt using 
```sh
rosrun rqt_gui rqt_gui
```

The tools can be found under Plugins -> Topics -> Message Publishers. The dropdown menu will provide the different tools under RobotNamespace/loop_closure_tools/(NAME OF SPECIFIC TOOL)

## add_factor

Allows the user to add a `BetweenEdge` to the graph that connects two keys in the isam2 pose graph.


`key1` and `key2` are unsigned integers that represent the keys of the two pose graph nodes to be connected.

Once the request has been made, the factor to be added is visualized by a yellow edge connecting the two keys (red spheres visualize these nodes). The user needs to confirm by running the acton confirmation boolean message (True = yes False = no). The message passing flow is as follows:

![Loop closure confirmation diagram](loop_closure_confirmation.png)

It is furthermore possible to define the attitude of the pose which is used to initialize the loop closure
optimization via the following parameterizations: (yaw is around the z axis)

```sh
<yaw> <pitch> and <roll>
```

Creates a loop closure between keys from_key and to_key with a rotation defined by
the given yaw, pitch and roll angles in radians.


## remove_factor

Allows the user to remove a `BetweenEdge` from the graph that connects two keys in the isam2 pose graph.

`key1` and `key2` are unsigned integers that represent the keys of the edge to be removed.

Once the request has been made, the factor to be removed is visualized by a yellow edge connecting the two keys (red spheres visualize these nodes). The user needs to confirm by running the acton confirmation boolean message (True = yes False = no). The message passing flow is analogous to the above.

## save_graph

Allows the user to save the entire pose graph, including all point clouds attached to it, to a zip file.

`filename.zip` is the path of the zip file which should be generated. Watch the output of the `blam_slam` ROS node for any error messages, or to learn the absolute path of the file that was generated. If a relative path name is given, it is typically saved with respect to the folder `~/.ros/`.

## load_graph

Allows the user to load the entire pose graph, including all point clouds attached to it, from a zip file.

`filename.zip` is the path of the zip file. The load_graph function will assume that a new robot is initialized, the position relative of the second robot can be modified in the parameters.yaml in Blam_slam

## restart

Allows the user to load the entire pose graph, including all point clouds attached to it, from `posegraph_backup.zip` in the case where LAMP fails, and one needs to restart it.

## batch_loopclosure

Will run a search for loopclosures through the entire posegraph. 