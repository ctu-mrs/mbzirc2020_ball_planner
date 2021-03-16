#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

PROJECT_NAME=just_flying

MAIN_DIR=~/"bag_files"

# following commands will be executed first, in each window
pre_input="export UAV_NAME=uav1; export RUN_TYPE=simulation; export UAV_TYPE=t650"

# define commands
# 'name' 'command'
input=(
  'Gazebo' "waitForRos; roslaunch ball_simulation simulation.launch gui:=false
"
  'Ball Attacher' "waitForRos; roslaunch ball_attacher attacher.launch
"
  'Spawn UAV1' "export UAV_NAME=uav1; waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth --enable-realsense-front
"
  'Spawn UAV2' "export UAV_NAME=uav2; waitForSimulation; spawn 2 --run --delete --enable-rangefinder --enable-ground-truth --enable-ball-holder
"
  'Control UAV1' "export UAV_NAME=uav1; waitForOdometry; roslaunch mrs_general core.launch config_constraint_manager:=custom_configs/constraint_manager.yaml config_uav_manager:=custom_configs/uav_manager.yaml config_gain_manager:=custom_configs/gain_manager.yaml config_mpc_tracker:=custom_configs/mpc_tracker.yaml
"
  'Control UAV2' "export UAV_NAME=uav2; waitForOdometry; roslaunch mrs_general core.launch config_constraint_manager:=custom_configs/constraint_manager.yaml config_uav_manager:=custom_configs/uav_manager.yaml config_gain_manager:=custom_configs/gain_manager.yaml config_mpc_tracker:=custom_configs/mpc_tracker.yaml
"
  'Prepare UAV1' "export UAV_NAME=uav1; waitForControl; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/control_manager/motors 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /$UAV_NAME/uav_manager/takeoff;
"
  'Prepare UAV2' "export UAV_NAME=uav2; waitForControl; rosservice call /uav2/mavros/cmd/arming 1; rosservice call /uav2/control_manager/motors 1; rosservice call /uav2/mavros/set_mode 0 offboard; rosservice call /uav2/uav_manager/takeoff;
rosservice call /uav2/control_manager/goto \"goal: [3.0, 3.0, 5.0, 0.6]\"
"
  'Trajectory' "waitForControl;
roslaunch ball_filter generate_eight.launch;
roslaunch trajectory_loader multimaster_trajectories_loader.launch loop:=true;
sleep 15;
roslaunch trajectory_loader multimaster_start_following.launch;
"
  'Detector' "waitForControl; roslaunch object_detect object_detect.launch
"
  'Filter' "waitForControl; roslaunch ball_filter filter_eightball.launch
"
  'Planner' "waitForControl; roslaunch ball_planner catch_eightball.launch
"
  'GoTo' "rosservice call /$UAV_NAME/control_manager/goto \"goal: [15.0, 15.0, 2.0, 0.0]\""
  'GoToRelative' "rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: [5.0, 5.0, 1.0, 3.14]\""
)

init_window="Control"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=$(tmux display-message -p "#S")

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  touch "$ITERATOR_FILE"
  ITERATOR="0"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest"
rm "$MAIN_DIR/latest"
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  /usr/bin/tmux new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

# add pane splitter for mrs_status
/usr/bin/tmux new-window -t $SESSION_NAME:$((${#names[*]}+1)) -n "mrs_status"

# clear mrs status file so that no clutter is displayed
truncate -s 0 /tmp/status.txt

# split all panes
pes=""
for ((i=0; i < ((${#names[*]}+2)); i++));
do
  pes=$pes"/usr/bin/tmux split-window -d -t $SESSION_NAME:$(($i))"
  pes=$pes"/usr/bin/tmux send-keys -t $SESSION_NAME:$(($i)) 'tail -F /tmp/status.txt'"
  pes=$pes"/usr/bin/tmux select-pane -U -t $(($i))"
done

/usr/bin/tmux send-keys -t $SESSION_NAME:$((${#names[*]}+1)) "${pes}"

sleep 6

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  /usr/bin/tmux pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  tmux send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

pes="sleep 1;"
for ((i=0; i < ((${#names[*]}+2)); i++));
do
  pes=$pes"/usr/bin/tmux select-window -t $SESSION_NAME:$(($i))"
  pes=$pes"/usr/bin/tmux resize-pane -U -t $(($i)) 150"
  pes=$pes"/usr/bin/tmux resize-pane -D -t $(($i)) 7"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

pes=$pes"/usr/bin/tmux select-window -t $SESSION_NAME:$init_index"
pes=$pes"waitForRos; roslaunch mrs_status f550.launch >> /tmp/status.txt"

/usr/bin/tmux send-keys -t $SESSION_NAME:$((${#names[*]}+1)) "${pes}"

/usr/bin/tmux -2 attach-session -t $SESSION_NAME

clear
