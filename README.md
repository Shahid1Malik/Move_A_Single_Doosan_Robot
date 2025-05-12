# Single Robot State Logger

## Overview

`single_robot_simple_py.py` is a ROS-based Python script for logging the state of a Doosan DSR robot (model A0509) to a CSV file. It:

- Subscribes to the robot's `/dsr01a0509/state` topic to receive `RobotState` messages.
- Extracts joint positions (X, Y, Z, Roll, Pitch, Yaw) at each callback.
- Records a timestamped snapshot of the robot’s Cartesian pose to a CSV file.
- Provides a clean shutdown service to stop the robot safely.

This tool is ideal for research and debugging, enabling easy analysis of robot kinematics over time.

## Features

- **ROS Integration**: Uses `rospy` for node management, topic subscription, and service proxies.
- **CSV Logging**: Appends header and state data to a specified CSV file.
- **Automated Shutdown**: Registers a `rospy.on_shutdown` callback to trigger an emergency stop.
- **Threaded Subscriber**: Runs the state listener in a daemon thread to keep the main loop free for control commands.

## Prerequisites

- **ROS Noetic** (or compatible ROS distribution)
- Python 3.x
- Doosan ROS Driver and common library in your workspace:
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/foo/doosan-robot.git
  git clone https://github.com/foo/common.git
  cd ..
  catkin_make
  source devel/setup.bash
  ```
- Python packages:
  - `rospy`
  - `csv`

## Configuration

Adjust the following variables in the script as needed:

| Variable           | Description                                 | Default Path / Value                               |
|--------------------|---------------------------------------------|----------------------------------------------------|
| `ROBOT_ID`         | Identifier for the robot                    | `"dsr01"`                                          |
| `ROBOT_MODEL`      | Robot model code                            | `"a0509"`                                          |
| `csv_file_path`    | Path to output CSV log file                 | `/home/lab_user/catkin_ws/src/.../robot_state_data.csv` |
| `logging_interval` | Callback write frequency (every N messages) | Configured via `msgRobotState_cb.count % 1`       |

## Installation

1. **Clone and build your workspace**:
   ```bash
   cd ~/catkin_ws/src
   # Clone this script into your ROS package
   git clone <your-repo-url>
   cd ..
   catkin_make
   source devel/setup.bash
   ```
2. **Ensure the script is executable**:
   ```bash
   chmod +x scripts/single_robot_simple_py.py
   ```

## Usage

1. **Launch ROS Master**:
   ```bash
   roscore
   ```
2. **Run the Script**:
   ```bash
   rosrun <your_package> single_robot_simple_py.py
   ```
3. **Monitor Output**:
   - Check terminal logs for robot status prints.
   - Open the CSV file to view recorded data:
     ```bash
     tail -f /path/to/robot_state_data.csv
     ```

## How It Works

1. **Initialization**:
   - Node `single_robot_simple_py` starts.
   - Registers `shutdown()` to call emergency stop via `pub_stop`.
   - Initializes Doosan driver with `DR_init` settings.
   - Starts a subscriber thread to listen to `/dsr01a0509/state`.
2. **Logging Callback** (`msgRobotState_cb`):
   - Increments message counter.
   - Logs robot state and Cartesian pose.
   - On first message (or defined interval), writes CSV header if file is new.
   - Appends timestamp (in microseconds) and pose vector to CSV.
3. **Main Loop**:
   - Sets robot mode to autonomous.
   - Configures global speed and acceleration.
   - Continuously commands the robot via `movel()` to move between defined waypoints.
4. **Shutdown**:
   - On node shutdown, publishes a quick stop to ensure safe robot halt.

## Troubleshooting

- **No CSV File Created**:
  - Verify write permissions for `csv_file_path`.
  - Ensure `msgRobotState_cb` is being called (check subscriber topic name).
- **Missing Header**:
  - Delete existing CSV to regenerate header on first run.
- **Robot Does Not Move**:
  - Confirm correct `ROBOT_ID` and `ROBOT_MODEL` match your Doosan setup.
  - Validate ROS service `/dsr01a0509/system/set_robot_mode` is available.
- **Script Errors on Import**:
  - Check `sys.path.append` points to the correct `common/imp` directory containing `DSR_ROBOT.py`.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Author

Shahid Shabeer Malik – Graduate Research Assistant, SLUAIR Lab, Saint Louis University

Contact: [shahid.malik@slu.edu](mailto:shahid.malik@slu.edu)

---

*Generated on May 12, 2025.*
