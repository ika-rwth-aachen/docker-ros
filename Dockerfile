ARG BASE_IMAGE=""
ARG PACKAGE_NAME=""
ARG COMMAND=""

############ DEPENDENCIES ###############
FROM ${BASE_IMAGE} as dependencies
ARG PACKAGE_NAME

ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE

COPY . src/${PACKAGE_NAME}

# get non apt dependencies
RUN if [[ -f "src/${PACKAGE_NAME}.repos" ]]; then \
        vcs import src < src/${PACKAGE_NAME}.repos && \
        rm src/${PACKAGE_NAME}.repos ; \
    fi

# get apt dependencies via rosdep
RUN apt-get update && \
    rosdep update && \
    ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH rosdep install -y --simulate --from-paths src --ignore-src \
        | tee $WORKSPACE/.install-dependencies.sh && \
    chmod +x $WORKSPACE/.install-dependencies.sh


############ DEPENDENCIES-INSTALL ########
FROM ${BASE_IMAGE} AS dependencies-install

ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE

COPY --from=dependencies $WORKSPACE/.install-dependencies.sh $WORKSPACE/.install-dependencies.sh

RUN apt-get update && \
    rosdep update && \
    $WORKSPACE/.install-dependencies.sh && \
    rm -rf /var/lib/apt/lists/*


############ DEVELOPMENT ################
FROM dependencies-install as development
ARG PACKAGE_NAME

# copy ROS packages
COPY . src/${PACKAGE_NAME}

# clone .repos
RUN if [[ -f "src/${PACKAGE_NAME}.repos" ]]; then \
        vcs import src < src/${PACKAGE_NAME}.repos && \
        rm src/${PACKAGE_NAME}.repos ; \
    fi

############ BUILD ######################
FROM development as build

# build ROS workspace
RUN catkin config --install --extend /opt/ros/$ROS_DISTRO && \
    catkin build -DCMAKE_BUILD_TYPE=Release --force-color --no-status --summarize 


############ RUN ######################
FROM dependencies-install as run
ARG COMMAND

# copy ROS packages
COPY --from=build $WORKSPACE/install install

RUN echo ${COMMAND} > cmd.sh && \
    chmod a+x cmd.sh

# run launchfile
COPY docker/docker-ros/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["./cmd.sh"]