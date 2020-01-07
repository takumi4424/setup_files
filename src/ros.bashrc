# ROS-melodic settings
source /opt/ros/melodic/setup.bash
export TURTLEBOT3_MODEL=burger
alias cm="catkin_make"
srcros () {
    search_path=${PWD}
    while :
    do
        if [ -f ${search_path}/devel/setup.bash ]; then
            source ${search_path}/devel/setup.bash
            break
        elif [ `realpath $search_path` = "/" ]; then
            break
        fi
        search_path="${search_path}/../"
    done
}
PYTHONWARNINGS=ignore::yaml.YAMLLoadWarning

