echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.zsh
this_script_dir=$(dirname "$0")
this_script_abs_dir=$(realpath $this_script_dir)
source $this_script_abs_dir/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
