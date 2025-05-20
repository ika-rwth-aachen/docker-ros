ARG BASE_IMAGE

############ dependencies ######################################################
FROM ${BASE_IMAGE} AS dependencies
ARG TARGETARCH
ENV TARGETARCH=${TARGETARCH}

USER root
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# create workspace folder structure
ENV WORKSPACE=/docker-ros/ws
WORKDIR $WORKSPACE
RUN mkdir -p src/target src/upstream src/downstream

# setup keys and sources.list for ROS packages
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
RUN test -n "$ROS_DISTRO" || (echo "missing build-arg: ROS_DISTRO" && false)
RUN apt-get update && \
    apt-get install -y curl gnupg && \
    if [[ "$ROS_DISTRO" == "noetic" ]] && ! cat /etc/apt/sources.list /etc/apt/sources.list.d/* | grep "http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"; then \
        apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
        echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list ; \
    elif ! cat /etc/apt/sources.list /etc/apt/sources.list.d/* | grep "http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"; then \
    	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# install ROS bootstrapping tools
ARG DISABLE_ROS_INSTALLATION="false"
RUN apt-get update && \
    apt-get install -y \
        git \
        python3-rosdep \
        python3-vcstool && \
    if [[ "$DISABLE_ROS_INSTALLATION" != "true" ]]; then \
        apt-get install -y ros-${ROS_DISTRO}-ros-core; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# copy contents of repository
COPY . src/target

# run custom script before dependency identification
ARG BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT="docker/before_dependency_identification.sh"
RUN if [[ -f src/target/${BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT} ]]; then \
        chmod +x src/target/${BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT} && \
        src/target/${BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT} ; \
    fi

# clone .repos upstream dependencies
ARG GIT_HTTPS_SERVER=
ARG GIT_HTTPS_USER=
ARG GIT_HTTPS_PASSWORD=
RUN if [[ -n ${GIT_HTTPS_SERVER} ]]; then \
        git config --global url.https://${GIT_HTTPS_USER}:${GIT_HTTPS_PASSWORD}@${GIT_HTTPS_SERVER}.insteadOf https://${GIT_HTTPS_SERVER} ; \
    fi
ARG GIT_SSH_PRIVATE_KEY=
ARG GIT_SSH_KNOWN_HOST_KEYS=
RUN if [[ -n ${GIT_SSH_PRIVATE_KEY} ]]; then \
        mkdir -p ~/.ssh && \
        echo -e "${GIT_SSH_PRIVATE_KEY}" > ~/.ssh/ssh_key && \
        chmod 400 ~/.ssh/ssh_key && \
        git config --global core.sshCommand "ssh -i ~/.ssh/ssh_key" && \
        touch ~/.ssh/known_hosts && \
        echo -e "${GIT_SSH_KNOWN_HOST_KEYS}" >> ~/.ssh/known_hosts ; \
    fi
COPY docker/docker-ros/docker/recursive_vcs_import.py /usr/local/bin
RUN apt-get update && \
    apt-get install -y python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

ARG VCS_IMPORT_FILE=".repos"
ARG ENABLE_RECURSIVE_VCS_IMPORT="true"
RUN if [[ $ENABLE_RECURSIVE_VCS_IMPORT == 'true' ]]; then \
        /usr/local/bin/recursive_vcs_import.py src src/upstream ; \
    elif [[ -f src/target/${VCS_IMPORT_FILE} ]]; then \
        vcs import --recursive src/upstream < src/target/${VCS_IMPORT_FILE} ; \
    fi

# remove blacklisted packages from workspace
ARG BLACKLISTED_PACKAGES_FILE="docker/blacklisted-packages.txt"
ARG ENABLE_RECURSIVE_BLACKLISTED_PACKAGES="false"
RUN echo "colcon list -p --base-paths src/ --packages-select \\" >> $WORKSPACE/.remove-packages.sh && \
    if [[ $ENABLE_RECURSIVE_BLACKLISTED_PACKAGES == 'true' ]]; then \
        find . -type f -name $(basename ${BLACKLISTED_PACKAGES_FILE}) -exec sed '$a\' {} \; | awk '{print "  " $0 " \\"}' >> $WORKSPACE/.remove-packages.sh ; \
    elif [[ -f src/target/${BLACKLISTED_PACKAGES_FILE} ]]; then \
        cat src/target/${BLACKLISTED_PACKAGES_FILE} | awk '{ gsub(/#.*/, ""); gsub(/^[ \t]+|[ \t]+$/, ""); if (NF) print "  " $0 " \\" }' >> $WORKSPACE/.remove-packages.sh ; \
    fi && \
    echo ";" >> $WORKSPACE/.remove-packages.sh && \
    chmod +x $WORKSPACE/.remove-packages.sh && \
    $WORKSPACE/.remove-packages.sh 2> /dev/null | xargs rm -rf

# create install script to run in dependencies-install stage
RUN echo "set -e" > $WORKSPACE/.install-dependencies.sh && \
    chmod +x $WORKSPACE/.install-dependencies.sh

# add custom script to run before dependency installation to install script
ARG BEFORE_DEPENDENCY_INSTALLATION_SCRIPT="docker/before_dependency_installation.sh"
ARG ENABLE_RECURSIVE_BEFORE_DEPENDENCY_INSTALLATION_SCRIPT="false"
RUN if [[ $ENABLE_RECURSIVE_BEFORE_DEPENDENCY_INSTALLATION_SCRIPT == 'true' ]]; then \
        find . -type f -name $(basename ${BEFORE_DEPENDENCY_INSTALLATION_SCRIPT}) -exec sed '$a\' {} >> $WORKSPACE/.install-dependencies.sh \; ; \
    elif [[ -f src/target/${BEFORE_DEPENDENCY_INSTALLATION_SCRIPT} ]]; then \
        cat src/target/${BEFORE_DEPENDENCY_INSTALLATION_SCRIPT} >> $WORKSPACE/.install-dependencies.sh && \
        echo "" >> $WORKSPACE/.install-dependencies.sh ; \
    fi

# add list of rosdep dependencies to install script
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get update && \
    (rosdep init || true) && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    export OS="ubuntu:$(lsb_release -c | awk '{print $2}')" && \
    if [[ "$ROS_DISTRO" = "rolling" && "$OS" = "ubuntu:focal" ]]; then export OS="ubuntu:jammy"; fi && \
    set -o pipefail && \
    PIP_BREAK_SYSTEM_PACKAGES=1 ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH rosdep install --os $OS -y --simulate --from-paths src --ignore-src \
    | sed -E "s/'apt-get install -y ([^']+)' \(alternative 1\/1\)/apt-get install -y \1/" \
    | tee -a $WORKSPACE/.install-dependencies.sh && \
    rm -rf /var/lib/apt/lists/*

# add additionally specified apt dependencies to install script
ARG ADDITIONAL_DEBS_FILE="docker/additional-debs.txt"
ARG ENABLE_RECURSIVE_ADDITIONAL_DEBS="false"
RUN echo "apt-get install -y \\" >> $WORKSPACE/.install-dependencies.sh && \
    set -o pipefail && \
    if [[ $ENABLE_RECURSIVE_ADDITIONAL_DEBS == 'true' ]]; then \
        find . -type f -name $(basename ${ADDITIONAL_DEBS_FILE}) -exec sed '$a\' {} \; | awk '{print "  " $0 " \\"}' >> $WORKSPACE/.install-dependencies.sh ; \
    elif [[ -f src/target/${ADDITIONAL_DEBS_FILE} ]]; then \
        cat src/target/${ADDITIONAL_DEBS_FILE} | awk '{ gsub(/#.*/, ""); gsub(/^[ \t]+|[ \t]+$/, ""); if (NF) print "  " $0 " \\" }' >> $WORKSPACE/.install-dependencies.sh ; \
    fi && \
    echo ";" >> $WORKSPACE/.install-dependencies.sh

# add additionally specified pip dependencies to install script
ARG ADDITIONAL_PIP_FILE="docker/additional-pip-requirements.txt"
ARG ENABLE_RECURSIVE_ADDITIONAL_PIP="false"
RUN echo "pip install pip \\" >> $WORKSPACE/.install-dependencies.sh && \
    set -o pipefail && \
    if [[ $ENABLE_RECURSIVE_ADDITIONAL_PIP == 'true' ]]; then \
        find . -type f -name $(basename ${ADDITIONAL_PIP_FILE}) -exec sed '$a\' {} \; | awk '{print "  " $0 " \\"}' >> $WORKSPACE/.install-dependencies.sh ; \
    elif [[ -f src/target/${ADDITIONAL_PIP_FILE} ]]; then \
        cat src/target/${ADDITIONAL_PIP_FILE} | awk '{ gsub(/#.*/, ""); gsub(/^[ \t]+|[ \t]+$/, ""); if (NF) print "  " $0 " \\" }' >> $WORKSPACE/.install-dependencies.sh ; \
    fi && \
    echo ";" >> $WORKSPACE/.install-dependencies.sh

# add custom script to run after dependency installation to install script
ARG AFTER_DEPENDENCY_INSTALLATION_SCRIPT="docker/custom.sh"
ARG ENABLE_RECURSIVE_AFTER_DEPENDENCY_INSTALLATION_SCRIPT="false"
RUN if [[ $ENABLE_RECURSIVE_AFTER_DEPENDENCY_INSTALLATION_SCRIPT == 'true' ]]; then \
        find . -type f -name $(basename ${AFTER_DEPENDENCY_INSTALLATION_SCRIPT}) -exec sed '$a\' {} >> $WORKSPACE/.install-dependencies.sh \; ; \
    elif [[ -f src/target/${AFTER_DEPENDENCY_INSTALLATION_SCRIPT} ]]; then \
        cat src/target/${AFTER_DEPENDENCY_INSTALLATION_SCRIPT} >> $WORKSPACE/.install-dependencies.sh ; \
    fi

# log install script
RUN cat $WORKSPACE/.install-dependencies.sh

# remove additional-files folder from copied repository content to avoid redundancies
ARG ADDITIONAL_FILES_DIR="docker/additional-files"
RUN rm -rf src/target/${ADDITIONAL_FILES_DIR}

############ dependencies-install ##############################################
FROM ${BASE_IMAGE} AS dependencies-install
ARG TARGETARCH
ARG GIT_HTTPS_URL
ARG GIT_HTTPS_USER
ARG GIT_HTTPS_PASSWORD
ENV TARGETARCH=${TARGETARCH}
ENV DOCKER_ROS=1

USER root
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# user setup
ENV DOCKER_USER=dockeruser
ENV DOCKER_UID=
ENV DOCKER_GID=

# ROS setup
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV WORKSPACE=/docker-ros/ws
ENV COLCON_HOME=$WORKSPACE/.colcon
WORKDIR $WORKSPACE

# setup keys and sources.list for ROS packages
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
RUN test -n "$ROS_DISTRO" || (echo "missing build-arg: ROS_DISTRO" && false)
RUN apt-get update && \
    apt-get install -y curl gnupg && \
    if [[ "$ROS_DISTRO" == "noetic" ]] && ! cat /etc/apt/sources.list /etc/apt/sources.list.d/* | grep "http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"; then \
        apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
        echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list ; \
    elif ! cat /etc/apt/sources.list /etc/apt/sources.list.d/* | grep "http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"; then \
    	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# copy contents of files-folder into image
ARG ADDITIONAL_FILES_DIR="docker/additional-files"
ADD ${ADDITIONAL_FILES_DIR}* /docker-ros/additional-files/

# install essential build tools and dependencies for install script
ARG DISABLE_ROS_INSTALLATION="false"
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        gosu \
        python-is-python3 \
        python3-pip && \
    if [[ "$DISABLE_ROS_INSTALLATION" != "true" ]]; then \
        apt-get install -y ros-${ROS_DISTRO}-ros-core; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# configure pip
RUN python -m pip config --global set global.break-system-packages true

# copy install script from dependencies stage
COPY --from=dependencies $WORKSPACE/.install-dependencies.sh $WORKSPACE/.install-dependencies.sh

# install dependencies
RUN apt-get update && \
    $WORKSPACE/.install-dependencies.sh && \
    rm -rf /var/lib/apt/lists/*

# install ROS CLI tools
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get update && \
    if [[ "$ROS_VERSION" == "1" ]]; then \
        apt-get install -y \
            python3-catkin-tools ; \
    elif [[ "$ROS_VERSION" == "2" ]]; then \
        apt-get install -y \
            python3-colcon-common-extensions ; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# source ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# install desired ROS 2 middleware
ARG RMW_IMPLEMENTATION
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    if [[ "$ROS_VERSION" == "2" && "$DISABLE_ROS_INSTALLATION" != "true" ]]; then \
        apt-get update && \
        RMW_PACKAGE=ros-$ROS_DISTRO-$(echo $RMW_IMPLEMENTATION | tr '_' '-') && \
        apt-get install -y $RMW_PACKAGE && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# move existing install space from base image to make room for new one
RUN if [[ -d $WORKSPACE/install ]]; then \
        mkdir -p /opt/ws_base_image && \
        mv $WORKSPACE/install /opt/ws_base_image/ && \
        echo "source /opt/ws_base_image/install/setup.bash" >> ~/.bashrc ; \
    fi

# set entrypoint
ENV TINI_VERSION=v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini-${TARGETARCH} /tini
RUN chmod +x /tini
COPY docker/docker-ros/docker/entrypoint.sh /
ENTRYPOINT ["/tini", "--", "/entrypoint.sh"]

############ dev ###############################################################
FROM dependencies-install AS dev

# copy contents of repository from dependencies stage
COPY --from=dependencies $WORKSPACE/src $WORKSPACE/src

CMD ["bash"]

############ build #############################################################
FROM dev AS build

ARG CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"
# build ROS workspace
RUN if [[ -x "$(command -v colcon)" ]]; then \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        [[ -f /opt/ws_base_image/install/setup.bash ]] && source /opt/ws_base_image/install/setup.bash ; \
        colcon build --cmake-args ${CMAKE_ARGS} ; \
    elif [[ -x "$(command -v catkin)" ]]; then \
        catkin config --install --extend /opt/ros/${ROS_DISTRO} && \
        catkin build -DCMAKE_BUILD_TYPE=Release --force-color --no-status --summarize ; \
    fi

############ run ###############################################################
FROM dependencies-install AS run

# remove source code, if still existing from custom.sh modifications
RUN rm -rf $WORKSPACE/src

# copy ROS install space from build stage
COPY --from=build $WORKSPACE/install install
RUN ldconfig
RUN echo "[[ -f $WORKSPACE/devel/setup.bash ]] && source $WORKSPACE/devel/setup.bash" >> ~/.bashrc && \
    echo "[[ -f $WORKSPACE/install/setup.bash ]] && source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# setup command
ARG COMMAND
ENV DEFAULT_CMD=${COMMAND}
CMD bash -c "${DEFAULT_CMD}"
