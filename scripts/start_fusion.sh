#!/bin/bash
# start_fusion.sh - Builds and / or runs the vin_fusion node
#
# Usage: bash scripts/start_fusion.sh [--build] [--start] [--help]
#   --build   clean build of vin_fusion only: removes vin_ws/build/vin_fusion
#             and vin_ws/install/vin_fusion, then
#             colcon build --packages-select vin_fusion in a clean shell, also
#             writing compile_commands.json for VS Code's code completion
#   --start   run the fusion node in this terminal; Ctrl-C stops it
#   --build --start   both, build first
#   no option shows this help
#
# Only vin_ws and only the vin_fusion package are touched; ros_ws and vio_ws
# are never built.

VIN_WS="/data/projects/udl_nav_vin/vin_ws"
PKG="vin_fusion"

show_help() {
    echo "Usage: bash $(basename "$0") [--build] [--start] [--help]"
    echo
    echo "  --build   remove $PKG's build and install folders in vin_ws and rebuild"
    echo "            only $PKG (colcon --packages-select)"
    echo "  --start   run the $PKG fusion node in this terminal"
    echo "  --help    show this help"
    echo
    echo "--build --start does both. With no option this help is shown and"
    echo "nothing is built or started."
}

build_pkg() {
    # Guard the rm: both paths must be the package folders inside vin_ws
    local build_dir="$VIN_WS/build/$PKG"
    local install_dir="$VIN_WS/install/$PKG"
    case "$build_dir$install_dir" in
        "$VIN_WS"/build/"$PKG""$VIN_WS"/install/"$PKG") ;;
        *) echo "refusing to remove unexpected paths"; exit 1 ;;
    esac
    echo "removing $build_dir and $install_dir"
    rm -rf -- "$build_dir" "$install_dir"

    # Clean shell so nothing sourced in this terminal (e.g. ros_ws) leaks in
    echo "building $PKG"
    env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="$TERM" \
        bash -c "source /opt/ros/humble/setup.bash && cd '$VIN_WS' && colcon build --packages-select $PKG --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "build failed (exit $rc)"
        exit $rc
    fi
    echo "build done"
}

start_node() {
    if [ ! -x "$VIN_WS/install/$PKG/lib/$PKG/fusion_node" ]; then
        echo "$PKG isn't built yet: run with --build first"
        exit 1
    fi
    source /opt/ros/humble/setup.bash
    source "$VIN_WS/install/setup.bash"
    exec ros2 run "$PKG" fusion_node
}

DO_BUILD=0
DO_START=0
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        --start) DO_START=1 ;;
        --help|-h) show_help; exit 0 ;;
        *) echo "Unknown option: $arg"; echo; show_help; exit 1 ;;
    esac
done

[ $DO_BUILD -eq 1 ] && build_pkg
[ $DO_START -eq 1 ] && start_node
exit 0
