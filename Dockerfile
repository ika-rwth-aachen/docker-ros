ARG BASE_IMAGE

############ DEPENDENCIES ###############
FROM ${BASE_IMAGE} as dependencies

ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE

COPY . src/

# move top-level package to package folder in src/
RUN if [[ -f "src/package.xml" ]]; then \
        PACKAGE_NAME=$(sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p' src/package.xml) && \
        mkdir -p src/${PACKAGE_NAME} && \
        cd src && shopt -s dotglob && find * -maxdepth 0 -not -name ${PACKAGE_NAME} -exec mv {} ${PACKAGE_NAME} \; ; \
    fi

# clone .repos
ARG GIT_HTTPS_URL=https://gitlab.ika.rwth-aachen.de
ARG GIT_HTTPS_USER=
ARG GIT_HTTPS_PASSWORD=
RUN if [ ! -z ${GIT_HTTPS_USER} ]; then \
        git config --global url.https://${GIT_HTTPS_USER}:${GIT_HTTPS_PASSWORD}@gitlab.ika.rwth-aachen.de.insteadOf ${GIT_HTTPS_URL} ; \
    fi
COPY docker/docker-ros/recursive_vcs_import.py /usr/local/bin
RUN cd src && \
    /usr/local/bin/recursive_vcs_import.py

# get apt dependencies via rosdep
RUN apt-get update && \
    rosdep update && \
    if [ -x "$(command -v colcon)" ]; then export OS="ubuntu:jammy"; else export OS="ubuntu:focal"; fi && \
    ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH rosdep install --os $OS -y --simulate --from-paths src --ignore-src \
        | tee $WORKSPACE/.install-dependencies.sh && \
    chmod +x $WORKSPACE/.install-dependencies.sh

# add additional apt dependencies
RUN echo "apt-get install -y \\" >> $WORKSPACE/.install-dependencies.sh && \
    find . -type f -name "additional.apt-dependencies" -exec cat {} \; | awk '{print "  " $0 " \\"}' >> $WORKSPACE/.install-dependencies.sh && \
    echo ";" >> $WORKSPACE/.install-dependencies.sh

# add custom installations
RUN find . -type f -name "custom.sh" -exec cat {} >> $WORKSPACE/.install-dependencies.sh \;

############ DEPENDENCIES-INSTALL ########
FROM ${BASE_IMAGE} AS dependencies-install

ENV WORKSPACE $DOCKER_HOME/ws
WORKDIR $WORKSPACE

COPY --from=dependencies $WORKSPACE/.install-dependencies.sh $WORKSPACE/.install-dependencies.sh

RUN apt-get update && \
    $WORKSPACE/.install-dependencies.sh && \
    rm -rf /var/lib/apt/lists/*

############ DEVELOPMENT ################
FROM dependencies-install as development

COPY . src/

# move top-level package to package folder in src/
RUN if [[ -f "src/package.xml" ]]; then \
        PACKAGE_NAME=$(sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p' src/package.xml) && \
        mkdir -p src/${PACKAGE_NAME} && \
        cd src && shopt -s dotglob && find * -maxdepth 0 -not -name ${PACKAGE_NAME} -exec mv {} ${PACKAGE_NAME} \; ; \
    fi

# clone .repos
ARG GIT_HTTPS_URL=https://gitlab.ika.rwth-aachen.de
ARG GIT_HTTPS_USER=
ARG GIT_HTTPS_PASSWORD=
RUN if [ ! -z ${GIT_HTTPS_USER} ]; then \
        git config --global url.https://${GIT_HTTPS_USER}:${GIT_HTTPS_PASSWORD}@gitlab.ika.rwth-aachen.de.insteadOf ${GIT_HTTPS_URL} ; \
    fi
COPY docker/docker-ros/recursive_vcs_import.py /usr/local/bin
RUN cd src && \
    /usr/local/bin/recursive_vcs_import.py

############ BUILD ######################
FROM development as build

# build ROS workspace
RUN if [ -x "$(command -v colcon)" ]; then \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release ; \
    elif [ -x "$(command -v catkin)" ]; then \
        catkin config --install --extend /opt/ros/${ROS_DISTRO} && \
        catkin build -DCMAKE_BUILD_TYPE=Release --force-color --no-status --summarize ; \
    fi

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
