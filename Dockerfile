ARG BASE_IMAGE

############ dependencies ######################################################
FROM ${BASE_IMAGE} as dependencies

# create workspace folder structure
ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE
RUN mkdir -p src/target src/upstream src/downstream

# copy contents of repository
COPY . src/repository

# if repository is a top-level package, move contents to <PACKAGE_NAME> folder
RUN shopt -s dotglob && \
    if [[ -f "src/repository/package.xml" ]]; then \
        PACKAGE_NAME=$(sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p' src/repository/package.xml) && \
        mkdir -p src/target/${PACKAGE_NAME} && \
        mv src/repository/* src/target/${PACKAGE_NAME} ; \
    else \
        mv src/repository/* src/target ; \
    fi && \
    rm -r src/repository

# clone .repos upstream dependencies
ARG GIT_HTTPS_URL=https://gitlab.ika.rwth-aachen.de
ARG GIT_HTTPS_USER=
ARG GIT_HTTPS_PASSWORD=
RUN if [ ! -z ${GIT_HTTPS_USER} ]; then \
        git config --global url.https://${GIT_HTTPS_USER}:${GIT_HTTPS_PASSWORD}@gitlab.ika.rwth-aachen.de.insteadOf ${GIT_HTTPS_URL} ; \
    fi
COPY docker/docker-ros/recursive_vcs_import.py /usr/local/bin
RUN /usr/local/bin/recursive_vcs_import.py src src/upstream

# create install script with list of rosdep dependencies
RUN apt-get update && \
    rosdep update && \
    if [ -x "$(command -v colcon)" ]; then export OS="ubuntu:jammy"; else export OS="ubuntu:focal"; fi && \
    ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH rosdep install --os $OS -y --simulate --from-paths src --ignore-src \
        | tee $WORKSPACE/.install-dependencies.sh && \
    chmod +x $WORKSPACE/.install-dependencies.sh

# add additionally specified apt dependencies to install script
RUN echo "apt-get install -y \\" >> $WORKSPACE/.install-dependencies.sh && \
    find . -type f -name "additional.apt-dependencies" -exec cat {} \; | awk '{print "  " $0 " \\"}' >> $WORKSPACE/.install-dependencies.sh && \
    echo ";" >> $WORKSPACE/.install-dependencies.sh

# add custom installation commands to install script
RUN find . -type f -name "custom.sh" -exec cat {} >> $WORKSPACE/.install-dependencies.sh \;

############ dependencies-install ##############################################
FROM ${BASE_IMAGE} AS dependencies-install
ARG TARGETARCH
ENV TARGETARCH=${TARGETARCH}

# set workspace
ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE

# copy contents of copy-folder into image, if it exists (use yaml as existing dummy)
COPY docker/docker-compose.yaml docker/copy* $DOCKER_HOME/copy/
RUN rm $DOCKER_HOME/copy/docker-compose.yaml

# copy install script from dependencies stage
COPY --from=dependencies $WORKSPACE/.install-dependencies.sh $WORKSPACE/.install-dependencies.sh

# install dependencies
RUN apt-get update && \
    $WORKSPACE/.install-dependencies.sh && \
    rm -rf /var/lib/apt/lists/*

############ development #######################################################
FROM dependencies-install as development

# copy contents of repository from dependencies stage
COPY --from=dependencies $WORKSPACE/src $WORKSPACE/src

############ build #############################################################
FROM development as build

# build ROS workspace
RUN if [ -x "$(command -v colcon)" ]; then \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release ; \
    elif [ -x "$(command -v catkin)" ]; then \
        catkin config --install --extend /opt/ros/${ROS_DISTRO} && \
        catkin build -DCMAKE_BUILD_TYPE=Release --force-color --no-status --summarize ; \
    fi

############ run ###############################################################
FROM dependencies-install as run

# copy ROS install space from build stage
COPY --from=build $WORKSPACE/install install

# setup entrypoint
ARG COMMAND
COPY docker/docker-ros/entrypoint.sh /
RUN echo ${COMMAND} > cmd.sh && \
    chmod a+x cmd.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["./cmd.sh"]
