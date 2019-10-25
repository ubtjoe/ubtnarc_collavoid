XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# user provides these arguments
export ROS_MASTER_URI=$1
export PATH_TO_SRC=$2
# end arguments

export CONT_USER_ID=collavoid
#export ROS_HOSTNAME=$(ifconfig wlan0 | awk '/inet / {print $2}')  # XXX for Jetson networking
export ROS_HOSTNAME=$(ifconfig wlp3s0 | awk '/inet / {print $2}')  # XXX for Jetson networking

docker run -i -t --rm \
    --env DISPLAY=$DISPLAY \
    --env "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env "XAUTHORITY=$XAUTH" \
    -v "$XAUTH:$XAUTH" \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev:/dev \
    -v $PATH_TO_SRC:/home/$CONT_USER_ID/catkin_ws/src/ubtnarc_collavoid \
    --name collavoid-deps-c \
    --privileged \
    --network=host \
    --env "ROS_MASTER_URI=http://$ROS_MASTER_URI:11311" \
    --env "ROS_HOSTNAME=$ROS_HOSTNAME" \
    collavoid-deps
