#!/bin/bash
# replay_run.sh - Replays a recorded bag into OpenVINS and records the run
#
# Usage: bash replay_run.sh <bag_name> <run_name> [--faults S] [--rate R] [--rviz]
#
# Starts OpenVINS with the sim config, starts record_run.py, plays the bag,
# then stops both. The run is saved to runs/eval/<run_name>/ like a live one,
# so it's scored the same way:  python3 score_run.py <run_name>
#
# With --faults, make_faulted_bag.py first writes a faulted copy of the bag to
# runs/bags/<bag_name>__<scenario>/ (reused if the scenario hasn't changed),
# and that copy is replayed instead, so faulted replays are repeatable too.
# S is a file in faults/ (with or without .yaml) or a path. The scenario and
# the fault log are copied into the run folder.
#
# The sim must not be running (its topics would mix with the bag's), and
# neither must another OpenVINS. Logs go to runs/replay_logs/<run_name>/.

VIN_DIR="/data/projects/udl_nav_vin"
VIO_WS="/data/projects/udl_aakash/vio_ws"
CONFIG_DIR="$VIN_DIR/vin_ws/config"
BAG_DIR="$VIN_DIR/runs/bags"
EVAL_DIR="$VIN_DIR/runs/eval"
HERE="$(cd "$(dirname "$0")" && pwd)"

show_help() {
    echo "Usage: bash $(basename "$0") <bag_name> <run_name> [--faults S] [--rate R] [--rviz]"
    echo
    echo "  bag_name    a bag in $BAG_DIR (made with record_bag.sh)"
    echo "  run_name    output folder in $EVAL_DIR, must not exist yet"
    echo "  --faults S  inject faults from scenario S (a file in $HERE/faults/, or a path)"
    echo "  --rate R    playback speed, default 1.0 (sim speed)"
    echo "  --rviz      open RViz with OpenVINS"
}

BAG_NAME=""
RUN_NAME=""
RATE="1.0"
RVIZ="false"
FAULTS=""
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) show_help; exit 0 ;;
        --rate) RATE="$2"; shift 2 ;;
        --faults) FAULTS="$2"; shift 2 ;;
        --rviz) RVIZ="true"; shift ;;
        *)
            if [ -z "$BAG_NAME" ]; then BAG_NAME="$1"
            elif [ -z "$RUN_NAME" ]; then RUN_NAME="$1"
            else echo "unexpected argument: $1"; exit 1
            fi
            shift ;;
    esac
done
if [ -z "$BAG_NAME" ] || [ -z "$RUN_NAME" ]; then
    show_help
    exit 0
fi

BAG="$BAG_DIR/$BAG_NAME"
LOG_DIR="$VIN_DIR/runs/replay_logs/$RUN_NAME"
if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "no bag at $BAG"
    exit 1
fi
# Which setup the bag was recorded with decides the IMU topic and the
# OpenVINS config: D455 copy (start_vio_sim.sh --d455) or the older /vio sim
if grep -q "name: /camera/camera/imu" "$BAG/metadata.yaml"; then
    IMU_TOPIC=/camera/camera/imu
    CONFIG="$CONFIG_DIR/openvins_d455/estimator_config.yaml"
else
    IMU_TOPIC=/vio/imu
    CONFIG="$CONFIG_DIR/openvins_sim/estimator_config.yaml"
fi
if [ -e "$EVAL_DIR/$RUN_NAME" ]; then
    echo "$EVAL_DIR/$RUN_NAME already exists, pick another run name"
    exit 1
fi
if pgrep -x run_subscribe_m > /dev/null; then
    echo "OpenVINS is already running, stop it first (pkill -x run_subscribe_m)"
    exit 1
fi
SCENARIO=""
if [ -n "$FAULTS" ]; then
    for f in "$FAULTS" "$HERE/faults/$FAULTS" "$HERE/faults/$FAULTS.yaml"; do
        if [ -f "$f" ]; then SCENARIO="$(realpath "$f")"; break; fi
    done
    if [ -z "$SCENARIO" ]; then
        echo "no fault scenario '$FAULTS' (looked in $HERE/faults/)"
        exit 1
    fi
fi

source /opt/ros/humble/setup.bash
source "$VIO_WS/install/setup.bash"

# A node that just exited (e.g. the injector from a previous replay) can stay
# listed for a few seconds, so give it time to drop off before refusing
imu_busy() {
    ros2 topic info /vio/imu 2>/dev/null | grep -q "Publisher count: [1-9]" ||
        ros2 topic info /camera/camera/imu 2>/dev/null | grep -q "Publisher count: [1-9]"
}
for _ in $(seq 10); do
    imu_busy || break
    sleep 1
done
if imu_busy; then
    echo "something is already publishing an IMU topic: stop the sim first (scripts/start_vio_sim.sh --stop)"
    exit 1
fi

mkdir -p "$LOG_DIR"

# With faults, write (or reuse) a faulted copy of the bag and replay that
if [ -n "$SCENARIO" ]; then
    FAULTED="$BAG_DIR/${BAG_NAME}__$(basename "$SCENARIO" .yaml)"
    echo "faulted bag: $FAULTED"
    if ! python3 "$HERE/make_faulted_bag.py" "$BAG" "$SCENARIO" "$FAULTED" > "$LOG_DIR/make_faulted_bag.log" 2>&1; then
        echo "couldn't make the faulted bag, see $LOG_DIR/make_faulted_bag.log"
        exit 1
    fi
    tail -n 1 "$LOG_DIR/make_faulted_bag.log"
    BAG="$FAULTED"
fi

OV_PID=""
REC_PID=""

stop_all() {
    if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
        kill -INT "$REC_PID"
        wait "$REC_PID" 2>/dev/null
    fi
    if [ -n "$OV_PID" ] && kill -0 "$OV_PID" 2>/dev/null; then
        kill -INT "$OV_PID"
        for _ in $(seq 20); do
            kill -0 "$OV_PID" 2>/dev/null || break
            sleep 0.5
        done
    fi
    # OpenVINS sometimes hangs on shutdown; it's ours, we checked none was running
    if pgrep -x run_subscribe_m > /dev/null; then
        pkill -INT -x run_subscribe_m
        sleep 3
        pkill -KILL -x run_subscribe_m 2>/dev/null
    fi
}
trap 'echo; echo "interrupted, stopping"; stop_all; exit 130' INT TERM

echo "starting OpenVINS with $(basename "$(dirname "$CONFIG")") (log: $LOG_DIR/openvins.log)"
ros2 launch ov_msckf subscribe.launch.py config_path:="$CONFIG" \
    max_cameras:=1 use_stereo:=false rviz_enable:="$RVIZ" \
    > "$LOG_DIR/openvins.log" 2>&1 &
OV_PID=$!

# wait until OpenVINS has subscribed to the IMU
for _ in $(seq 30); do
    ros2 topic info "$IMU_TOPIC" 2>/dev/null | grep -q "Subscription count: [1-9]" && break
    kill -0 "$OV_PID" 2>/dev/null || break
    sleep 1
done
if ! ros2 topic info "$IMU_TOPIC" 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    echo "OpenVINS didn't come up, see $LOG_DIR/openvins.log"
    stop_all
    exit 1
fi

echo "starting recorder (log: $LOG_DIR/recorder.log)"
python3 "$HERE/record_run.py" --name "$RUN_NAME" > "$LOG_DIR/recorder.log" 2>&1 &
REC_PID=$!
sleep 2

echo "playing $BAG at rate $RATE"
ros2 bag play "$BAG" --rate "$RATE" --disable-keyboard-controls \
    --qos-profile-overrides-path "$HERE/qos_overrides.yaml" \
    > "$LOG_DIR/play.log" 2>&1

# let the last messages through before stopping
sleep 3
stop_all
tail -n 1 "$LOG_DIR/recorder.log"
if [ -n "$SCENARIO" ] && [ -d "$EVAL_DIR/$RUN_NAME" ]; then
    cp "$SCENARIO" "$EVAL_DIR/$RUN_NAME/fault_scenario.yaml"
    cp "$BAG/faults.csv" "$EVAL_DIR/$RUN_NAME/faults.csv" 2>/dev/null
    echo "faults:"
    grep -v "^t," "$BAG/faults.csv" 2>/dev/null | grep -v ",count," | sed 's/^/  /'
fi
echo
echo "score it with:  python3 $HERE/score_run.py $RUN_NAME"
