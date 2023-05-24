# sweep-ros

Port of the [sweep-ros](https://github.com/scanse/sweep-ros) node to ROS2.

## Dependencies
The Sweep ROS Driver and Node requires that the libsweep library from the [sweep-sdk](https://github.com/scanse/sweep-sdk) be installed on the computer. I've included it as a submodule in this repository for ease of use.

## Scanse Sweep ROS Driver and Node

This node is currently publishing a `pointcloud2` msg. This is because the Sweep device does not use fixed azimuth intervals. For more details see the Sweep [theory of operation](https://support.scanse.io/hc/en-us/articles/115006333327-Theory-of-Operation). Use `sweep2scan.launch` for conversion to `laserscan` msg.

## Firmware Compatibility:
Currently, sweep-ros is only compatible with sweep `firmware v1.1` or greater.

You can check the firmware version installed on your sweep device by using a serial terminal (see [manual](https://s3.amazonaws.com/scanse/Sweep_user_manual.pdf)) or more easily using the sweep visualizer (see [instructions](https://support.scanse.io/hc/en-us/articles/224557908-Upgrading-Firmware)).