#! /usr/bin/env bash

grep_ip()
{
    grep -Eo '[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' $@
}

ip_address()
{
    if [ $# -eq 0 ];
    then
        echo "Usage: $ ip_address network_name" >&2
    fi
    ip a | grep $@ | grep_ip | head -n 1
}

rbot_remote()
{
    if [ $# -eq 0 ];
    then
        echo "Usage: $ rbot_remote docker_name [docker_network_name]" >&2
    fi

    local self_addr=$(ip_address ${2-docker0})
    export GAZEBO_IP=$self_addr
    export ROS_IP=$self_addr

    local master_addr=$(docker container inspect $1 | grep '"IPAddress' | head -n 1 | grep_ip)
    rbot_set_master $master_addr ${@:3}
}

rbot_docker()
{
    local self_addr=$(ip_address eth0)
    export GAZEBO_IP=$self_addr
    export ROS_IP=$self_addr
    rbot_set_master ${1-$self_addr} ${@:2}
}

rbot_set_master()
{
    local master_addr=${1-localhost}
    export GAZEBO_MASTER_URI=http://$master_addr:${2-11345}
    export ROS_MASTER_URI=http://$master_addr:${3-11311}
}

rbot_init()
{
    grep -q docker /proc/self/cgroup && rbot_docker
    echo "Init done" > /dev/null
}

PKG_PATH=$(rospack find rbot_sim_integration)

PKG_GAZEBO_BASE=$PKG_PATH/models/gazebo

gazebo_version()
{
    gzserver -v | head -1 | grep --only-matching '[0-9]\+' | head -1
}
export GAZEBO_MODEL_PATH=/usr/share/gazebo-`gazebo_version`/models:$PKG_GAZEBO_BASE/model
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-`gazebo_version`/:$PKG_GAZEBO_BASE
export GAZEBO_PLUGIN_PATH=~/.gazebo:$(roscd;pwd)/lib
