# ROS-melodic settings
if [ -f /opt/ros/melodic/setup.bash ]; then
	source /opt/ros/melodic/setup.bash
fi
alias killgazebo="killall -9 gazebo & killall -9 gzserver & killall -9 gzclient"
function find_catkin_ws () {
	curdir=${PWD}
	while :
	do
		if [ -f ${curdir}/.catkin_workspace ]; then
			echo ${curdir}
			break
		elif [ ${curdir} = "/" ]; then
			break
		else
			curdir=`realpath ${curdir}/../`
		fi
	done
}
function cm () {
	catkin_ws=$(find_catkin_ws)
	if [ -z ${catkin_ws} ]; then
		echo "catkin workspace not found."
	else
		(cd ${catkin_ws} && catkin_make)
	fi
}
function srcros () {
	catkin_ws=$(find_catkin_ws)
	if [ -z ${catkin_ws} ]; then
		echo "catkin workspace not found."
	elif [ ! -f ${catkin_ws}/devel/setup.bash ]; then
		echo "\$CATKIN_WS/devel/setup.bash not found"
	else
		source ${catkin_ws}/devel/setup.bash
	fi
}
function cmrm () {
	catkin_ws=$(find_catkin_ws)
	if [ -z ${catkin_ws} ]; then
		echo "catkin workspace not found."
	else
		rm -rf ${catkin_ws}/devel ${catkin_ws}/build
	fi
}

