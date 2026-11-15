#!/bin/bash
# record_bag.sh - Records the VIO sim topics to a ROS 2 bag, for replay later
#
# Usage: bash record_bag.sh <name>
#
# Start it with the sim running and the drone still on the ground (OpenVINS
# initialises on the takeoff, so the bag has to include the still part before
# it). Stop it with Ctrl-C after landing. Saves to runs/bags/<name>/.
#
# Messages are stamped with sim time (--use-sim-time), so a replay runs at sim
# speed, not at the slower speed the live sim ran at.

VIN_DIR="/data/projects/vin_autonomy"
BAG_DIR="$VIN_DIR/runs/bags"
HERE="$(cd "$(dirname "$0")" && pwd)"
TOPICS=(
    /vio/camera/image
    /vio/camera/camera_info
    /vio/imu
    /vio/ground_truth/odom
    /mavros/state
)

show_help() {
    echo "Usage: bash $(basename "$0") <name>"
    echo
    echo "  Records ${TOPICS[*]}"
    echo "  to $BAG_DIR/<name>/. Ctrl-C to stop."
}

NAME="$1"
if [ -z "$NAME" ] || [ "$NAME" = "-h" ] || [ "$NAME" = "--help" ]; then
    show_help
    exit 0
fi

OUT="$BAG_DIR/$NAME"
if [ -e "$OUT" ]; then
    echo "$OUT already exists, pick another name"
    exit 1
fi

source /opt/ros/humble/setup.bash

# --use-sim-time needs /clock, which the sim's ros_gz bridge publishes
if ! ros2 topic info /clock 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "nothing is publishing /clock: start the sim first (scripts/start_vio_sim.sh --start)"
    exit 1
fi

mkdir -p "$BAG_DIR"
echo "recording to $OUT (Ctrl-C to stop)"
exec ros2 bag record --use-sim-time \
    --qos-profile-overrides-path "$HERE/qos_overrides.yaml" \
    -o "$OUT" "${TOPICS[@]}"
