#! /usr/bin/env bash

# @NOTE: Modify this variable to the correct location
source $HOME/catkin_ws/devel/setup.bash
#            ^^^^^^^^^  only this part needs to be modified (usually)

# Calls catkin_make from the correct location
cm()
{
    local x=`pwd`
    roscd; cd ..
    catkin_make $@
    local ret=$?
    cd $x
    return $ret
}

# Calls catkin_make from the correct location
cmi()
{
    local x=`pwd`
    roscd; cd ..
    catkin_make_isolated $@
    local ret=$?
    cd $x
    return $ret
}

# Runs the commands with a fake buffer. Ideal for running without display server
virtual_frame_buffer_launch()
{
    xvfb-run -s "-screen 0 1280x1024x24" $@
}

# Kills by process name compared to pkill by process id
pskill()
{
    local process=`ps aux | grep $@ | awk '{print $2}'`
    echo "Killing process(es) with ID:"
    echo "$process" | tee /dev/tty | xargs kill -9
}

# Kills all local gazebo process
gzkill()
{
    pskill gzserver
    pskill gazebo
    pskill gzclient
}

# Kills all local ros and gazebo process. Use with cautions
roskill()
{
    gzkill
    pskill ros
}
