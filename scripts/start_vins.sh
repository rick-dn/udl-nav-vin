#!/bin/bash
# start_vins.sh - Starts / stops OpenVINS with the sim config and RViz
#
# Usage: bash scripts/start_vins.sh [--start [--no-rviz] | --stop | --help]
#   --start   run OpenVINS (ov_msckf) in this terminal with RViz; Ctrl-C stops it
#   --start --no-rviz
#             same without RViz
#   --stop    stop a running OpenVINS (from another terminal)
#   no option shows this help
#
# Start it with the drone still on the ground: OpenVINS initialises on the
# takeoff. The sim (start_vio_sim.sh) or a bag replay must provide the camera
# and IMU topics.
#
# The config is picked from whichever IMU topic is being published:
# /camera/camera/imu (start_vio_sim.sh --d455) -> config/openvins_d455,
# /vio/imu (the older sim) -> config/openvins_sim. With neither, D455.

VIO_WS="/data/projects/udl_aakash/vio_ws"
CONFIG_DIR="/data/projects/udl_nav_vin/vin_ws/config"

show_help() {
    echo "Usage: bash $(basename "$0") [--start [--no-rviz] | --stop | --help]"
    echo
    echo "  --start            run OpenVINS with the sim config and RViz in this terminal"
    echo "  --start --no-rviz  same without RViz"
    echo "  --stop             stop a running OpenVINS"
    echo "  --help             show this help"
    echo
    echo "With no option this help is shown and nothing is started."
}

start_vins() {
    local rviz="true"
    [ "$1" = "--no-rviz" ] && rviz="false"
    if pgrep -x run_subscribe_m > /dev/null; then
        echo "OpenVINS is already running (stop it with: bash $(basename "$0") --stop)"
        exit 1
    fi
    source /opt/ros/humble/setup.bash
    source "$VIO_WS/install/setup.bash"
    local config="$CONFIG_DIR/openvins_d455/estimator_config.yaml"
    if ros2 topic info /vio/imu 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        config="$CONFIG_DIR/openvins_sim/estimator_config.yaml"
    fi
    echo "OpenVINS config: $config"
    exec ros2 launch ov_msckf subscribe.launch.py config_path:="$config" \
        max_cameras:=1 use_stereo:=false rviz_enable:="$rviz"
}

stop_vins() {
    if ! pgrep -x run_subscribe_m > /dev/null; then
        echo "OpenVINS is not running."
        return 0
    fi
    pkill -INT -x run_subscribe_m
    for _ in $(seq 10); do
        pgrep -x run_subscribe_m > /dev/null || { echo "OpenVINS stopped."; return 0; }
        sleep 1
    done
    # it sometimes hangs on shutdown
    pkill -KILL -x run_subscribe_m
    echo "OpenVINS killed (it didn't stop on Ctrl-C)."
}

case "$1" in
    --start)
        case "$2" in
            ""|--no-rviz) start_vins "$2" ;;
            *) echo "Unknown option: $2"; echo; show_help; exit 1 ;;
        esac ;;
    --stop) stop_vins ;;
    --help|-h|"") show_help ;;
    *) echo "Unknown option: $1"; echo; show_help; exit 1 ;;
esac
