#!/bin/bash
set -e

# source ROS workspace
source /opt/ros/$ROS_DISTRO/setup.bash
[[ -f /opt/ws_rmw_zenoh/install/setup.bash ]] && source /opt/ws_rmw_zenoh/install/setup.bash
[[ -f /opt/ws_base_image/install/setup.bash ]] && source /opt/ws_base_image/install/setup.bash
[[ -f $WORKSPACE/devel/setup.bash ]] && source $WORKSPACE/devel/setup.bash
[[ -f $WORKSPACE/install/setup.bash ]] && source $WORKSPACE/install/setup.bash

# exec as dockeruser with configured UID/GID
if [[ $DOCKER_UID && $DOCKER_GID ]]; then
    if ! getent group $DOCKER_GID > /dev/null 2>&1; then
        groupadd -g $DOCKER_GID $DOCKER_USER
    else
        echo -e "\e[33mWARNING | Cannot create group '$DOCKER_USER' with GID $DOCKER_GID, another group '$(getent group $DOCKER_GID | cut -d: -f1)' with same GID is already existing\e[0m"
    fi
    if ! getent passwd $DOCKER_UID > /dev/null 2>&1; then
        useradd -s /bin/bash \
                -u $DOCKER_UID \
                -g $DOCKER_GID \
                --create-home \
                --home-dir /home/$DOCKER_USER \
                --groups sudo,video \
                --password "$(openssl passwd -1 $DOCKER_USER)" \
                $DOCKER_USER && \
                touch /home/$DOCKER_USER/.sudo_as_admin_successful
        cp /root/.bashrc /home/$DOCKER_USER
        ln -s $WORKSPACE /home/$DOCKER_USER/ws
        chown -h $DOCKER_UID:$DOCKER_GID $WORKSPACE /home/$DOCKER_USER/ws /home/$DOCKER_USER/.sudo_as_admin_successful
        if [[ -d $WORKSPACE/src ]]; then
            chown -R $DOCKER_UID:$DOCKER_GID $WORKSPACE/src
        fi
    else
        echo -e "\e[33mWARNING | Cannot create user '$DOCKER_USER' with UID $DOCKER_UID, another user '$(getent passwd $DOCKER_UID | cut -d: -f1)' with same UID is already existing\e[0m"
    fi
    [[ $(pwd) == "$WORKSPACE" ]] && cd /home/$DOCKER_USER/ws
    exec gosu $DOCKER_USER "$@"
else
    exec "$@"
fi
