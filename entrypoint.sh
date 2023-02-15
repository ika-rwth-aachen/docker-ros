#!/bin/bash
set -e

# source ROS workspace
source /opt/ros/$ROS_DISTRO/setup.bash
[[ -f $WORKSPACE/devel/setup.bash ]] && source $WORKSPACE/devel/setup.bash
[[ -f $WORKSPACE/install/setup.bash ]] && source $WORKSPACE/install/setup.bash

# exec as dockeruser with configured UID/GID
if [[ $DOCKER_UID && $DOCKER_GID ]]; then
    groupadd -g $DOCKER_GID $DOCKER_USER
    useradd -s /bin/bash \
            -u $DOCKER_UID \
            -g $DOCKER_USER \
            --create-home \
            --home-dir /home/$DOCKER_USER \
            --groups sudo,video \
            --password "$(openssl passwd -1 $DOCKER_USER)" \
            $DOCKER_USER && \
            touch /home/$DOCKER_USER/.sudo_as_admin_successful
    chown -R $DOCKER_USER:$DOCKER_USER $WORKSPACE
    ln -s $WORKSPACE /home/$DOCKER_USER/ws
    cd /home/$DOCKER_USER/ws
    exec gosu $DOCKER_USER "$@"
else
    exec "$@"
fi