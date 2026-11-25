#!/bin/bash
# start_vio_sim.sh - Opens terminals for the VIO SITL stack
# (iris_with_vio_camera in iris_warehouse_vio.sdf)
#
# Usage: bash scripts/start_vio_sim.sh [--start [--sensors | --d455] | --stop | --help]
#   --start   open the Gazebo, ArduPilot, MAVROS, Bridge and IMU Bridge tabs
#   --start --sensors
#             same, but with iris_with_vio_sensors in
#             iris_warehouse_vio_sensors.sdf (adds GNSS, magnetometer,
#             barometer, rangefinder) and a Sensor Bridge tab
#   --start --d455
#             the real-setup copy: iris_with_d455 in iris_warehouse_d455.sdf.
#             RealSense D455 topics (/camera/camera/...), rangefinder on
#             /rangefinder/range, ground truth on /sim/ground_truth/odom, and
#             MAVROS on sim time so ArduPilot's mag, baro and GNSS line up
#   --stop    stop the sim (also stops start_all.sh) and close its tabs
#   no option shows this help, so running the script by itself never
#   launches or builds anything
#
# The same script lives in udl_nav_vin/scripts and udl_aakash/scripts.
# ArduPilot runs from udl_nav_vin/runs/sitl so its eeprom.bin and logs stay
# separate from udl_aakash's start_all.sh and gnc_autonomy's runs.
# The ArduPilot, Gazebo and MAVROS installs are udl_aakash's, used as-is.

PROJECT_DIR="/data/projects/udl_aakash"
ARDUPILOT_DIR="/data/projects/udl_aakash/tools/ardupilot/Tools/autotest"
VIN_DIR="/data/projects/udl_nav_vin"
RUN_DIR="$VIN_DIR/runs/sitl"
IMU_BRIDGE="$VIN_DIR/vin_ws/src/gz_imu_bridge/gz_imu_bridge.py"
SENSOR_BRIDGE="$VIN_DIR/vin_ws/src/gz_imu_bridge/gz_sensor_bridge.py"
RANGE_BRIDGE="$VIN_DIR/vin_ws/src/gz_imu_bridge/gz_range_bridge.py"

show_help() {
    echo "Usage: bash $(basename "$0") [--start [--sensors | --d455] | --stop | --help]"
    echo
    echo "  --start   open the VIO sim in terminal tabs:"
    echo "            Gazebo (iris_warehouse_vio.sdf), ArduPilot SITL (--no-rebuild),"
    echo "            MAVROS, ros_gz bridge (camera, ground truth, clock) and IMU bridge"
    echo "  --start --sensors"
    echo "            same with iris_warehouse_vio_sensors.sdf, which adds GNSS,"
    echo "            magnetometer, barometer and rangefinder, plus a Sensor Bridge tab"
    echo "  --start --d455"
    echo "            the real-setup copy: D455 camera and IMU on the RealSense topic"
    echo "            names, rangefinder, ArduPilot's own mag/baro/GNSS via MAVROS"
    echo "            (on sim time), ground truth on /sim/ground_truth/odom"
    echo "  --stop    stop the sim and close its tabs: Gazebo, ArduPilot, MAVProxy,"
    echo "            MAVROS, bridges and rqt_image_view (also stops start_all.sh)"
    echo "  --help    show this help"
    echo
    echo "With no option this help is shown and nothing is started or built."
}

start_sim() {
    local mode="$1"
    local world="iris_warehouse_vio.sdf"
    [ "$mode" = "--sensors" ] && world="iris_warehouse_vio_sensors.sdf"
    [ "$mode" = "--d455" ] && world="iris_warehouse_d455.sdf"
    mkdir -p "$RUN_DIR"

    # Tab 1: Gazebo
    gnome-terminal --tab --title="Gazebo" -- bash -c "cd $PROJECT_DIR && gz sim -v4 -r $world; bash"

    # Small delay so the world is loaded before SITL connects to the plugin
    sleep 5

    # Tab 2: ArduPilot (--no-rebuild: use the existing build, never compile)
    # With --d455, ArduPilot's mag / baro / GNSS are estimator inputs, so
    # MAVProxy asks for 20 Hz streams instead of its default 4 Hz
    local ap_extra=""
    [ "$mode" = "--d455" ] && ap_extra="--mavproxy-args=--streamrate=20"
    gnome-terminal --tab --title="ArduPilot" -- bash -c "cd $RUN_DIR && $ARDUPILOT_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --map --console -L Basingstoke $ap_extra; bash"

    # Small delay before starting dependent services
    sleep 3

    # Tab 3: MAVROS. With --d455 its sensor topics (mag, baro, GNSS) are
    # estimator inputs, so it runs on sim time to match the camera and IMU.
    local mavros_args="-p system_id:=200 -p fcu_url:=udp://:14550@localhost:14555"
    [ "$mode" = "--d455" ] && mavros_args="$mavros_args -p use_sim_time:=true"
    gnome-terminal --tab --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args $mavros_args; bash"

    # Tab 4: Bridge (camera, ground truth, sim clock)
    # The IMU is not bridged here: this ros_gz_bridge is built for Gazebo
    # Garden and corrupts Harmonic IMU messages. Tab 5 handles it instead.
    if [ "$mode" = "--d455" ]; then
        gnome-terminal --tab --title="Bridge" -- bash -c "ros2 run ros_gz_bridge parameter_bridge \
/camera/camera/infra1/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image \
/camera/camera/infra1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/sim/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock; bash"
    else
        gnome-terminal --tab --title="Bridge" -- bash -c "ros2 run ros_gz_bridge parameter_bridge \
/vio/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
/vio/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/vio/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock; bash"
    fi

    # Tab 5: IMU bridge (Harmonic IMU -> ROS 2, see gz_imu_bridge.py)
    if [ "$mode" = "--d455" ]; then
        gnome-terminal --tab --title="IMU Bridge" -- bash -c "python3 $IMU_BRIDGE /camera/camera/imu /camera/camera/imu; bash"
    else
        gnome-terminal --tab --title="IMU Bridge" -- bash -c "python3 $IMU_BRIDGE; bash"
    fi

    # Tab 6: --sensors: GNSS, mag, baro, rangefinder; --d455: rangefinder only
    if [ "$mode" = "--sensors" ]; then
        gnome-terminal --tab --title="Sensor Bridge" -- bash -c "python3 $SENSOR_BRIDGE; bash"
    elif [ "$mode" = "--d455" ]; then
        gnome-terminal --tab --title="Range Bridge" -- bash -c "python3 $RANGE_BRIDGE; bash"
    fi
}

# Stop logic below is the same as udl_aakash/scripts/start_all.sh --stop

# Programs the sim tabs run (matched against the full command line)
SIM_PATTERN='^gz sim|sim_vehicle\.py|mavproxy\.py|bin/arducopter|mavros_node|parameter_bridge|rqt_image_view|gz_imu_bridge\.py|gz_sensor_bridge\.py|gz_range_bridge\.py'
# Same programs as they appear inside a tab's "bash -c" command
TAB_PATTERN='gz sim|sim_vehicle\.py|mavros_node|parameter_bridge|rqt_image_view|gz_imu_bridge\.py|gz_sensor_bridge\.py|gz_range_bridge\.py'

# PIDs matching the pattern whose program really is a sim program. A process
# that only mentions these names (a grep, an editor, another shell) is skipped.
find_programs() {
    local argv0 argv1
    for pid in $(pgrep -f "$SIM_PATTERN"); do
        [ "$pid" = "$$" ] && continue
        { IFS= read -r -d '' argv0; IFS= read -r -d '' argv1; } < /proc/$pid/cmdline 2>/dev/null || continue
        argv0="${argv0%% *}"  # gz renames itself "gz sim server", keep the first word
        case "${argv0##*/}" in
            gz|arducopter|parameter_bridge|mavros_node) echo "$pid" ;;
            python|python3)
                case "${argv1##*/}" in
                    sim_vehicle.py|mavproxy.py|ros2|gz_imu_bridge.py|gz_sensor_bridge.py|gz_range_bridge.py|rqt_image_view) echo "$pid" ;;
                esac ;;
        esac
    done
}

# The "bash -c '<program>; bash'" shells that hold each terminal tab open
find_tab_shells() {
    for pid in $(pgrep -f "^bash -c .*($TAB_PATTERN)"); do
        [ "$pid" = "$$" ] || echo "$pid"
    done
}

# Wait up to $1 seconds for the given PIDs to exit
wait_for_exit() {
    local secs=$1; shift
    for _ in $(seq "$secs"); do
        kill -0 "$@" 2>/dev/null || return 0
        sleep 1
    done
    return 1
}

stop_sim() {
    # Note the tab shells first: once their program exits they turn into a
    # plain "bash" and can no longer be told apart from other terminals
    local tabs progs
    tabs=$(find_tab_shells)
    progs=$(find_programs)

    if [ -z "$progs" ] && [ -z "$tabs" ]; then
        echo "Nothing running."
        return 0
    fi

    if [ -n "$progs" ]; then
        echo "Stopping:"
        for pid in $progs; do
            echo "  $pid $(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-80)"
        done

        # Ctrl-C first so everything shuts down cleanly
        kill -INT $progs 2>/dev/null
        if ! wait_for_exit 15 $progs; then
            progs=$(find_programs)
            echo "Still running after Ctrl-C, sending TERM:" $progs
            kill -TERM $progs 2>/dev/null
            if ! wait_for_exit 10 $progs; then
                progs=$(find_programs)
                echo "Still running after TERM, sending KILL:" $progs
                kill -KILL $progs 2>/dev/null
            fi
        fi
    fi

    # Close the terminal tabs
    if [ -n "$tabs" ]; then
        kill -HUP $tabs 2>/dev/null
        wait_for_exit 5 $tabs
    fi

    progs=$(find_programs)
    if [ -n "$progs" ]; then
        echo "Could not stop:" $progs
        return 1
    fi
    echo "Sim stopped."
}

case "$1" in
    --start)
        case "$2" in
            ""|--sensors|--d455) start_sim "$2" ;;
            *) echo "Unknown option: $2"; echo; show_help; exit 1 ;;
        esac ;;
    --stop)  stop_sim ;;
    --help|-h|"") show_help ;;
    *) echo "Unknown option: $1"; echo; show_help; exit 1 ;;
esac
