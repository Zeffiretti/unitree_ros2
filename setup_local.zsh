echo "Setup unitree ros2 simulation environment"
SYS_VERSION=$(lsb_release -rs)
if [ $SYS_VERSION = "20.04" ]; then
    ros_distro="foxy"
elif [ $SYS_VERSION = "22.04" ]; then
    ros_distro="humble"
else
    echo "Unsupported system version: $SYS_VERSION"
    return
fi
echo "ROS distro: $ros_distro"
source /opt/ros/$ros_distro/setup.zsh
this_script_dir=$(dirname "$0")
this_script_abs_dir=$(realpath $this_script_dir)
source $this_script_abs_dir/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
