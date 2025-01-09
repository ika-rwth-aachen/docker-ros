<img src="https://github.com/ika-rwth-aachen/docker-ros/raw/main/assets/logo.png" height=130 align="right">

# *docker-ros* – Automated Containerization of ROS Apps

<p align="center">
  <img src="https://img.shields.io/github/v/release/ika-rwth-aachen/docker-ros"/></a>
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/docker-ros"/>
  <a href="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/github.yml"><img src="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/github.yml/badge.svg"/></a>
  <a href="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/gitlab.yml"><img src="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/gitlab.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS-noetic-blueviolet"/>
  <img src="https://img.shields.io/badge/ROS 2-foxy|humble|iron|jazzy-blueviolet"/>
</p>

*docker-ros* automatically builds minimal container images of ROS applications.

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **DevOps, Containerization and Orchestration of Software-Defined Vehicles** are some of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***

- [About](#about)
  - [Prerequisites](#prerequisites)
- [Usage](#usage)
  - [Build a minimal image for deployment](#build-a-minimal-image-for-deployment)
  - [Build development and deployment images](#build-development-and-deployment-images)
  - [Build multi-arch images](#build-multi-arch-images)
  - [Build deployment image with additional industrial\_ci check](#build-deployment-image-with-additional-industrial_ci-check)
  - [Build multi-arch images on arch-specific self-hosted runners in parallel](#build-multi-arch-images-on-arch-specific-self-hosted-runners-in-parallel)
  - [Build images locally](#build-images-locally)
- [Advanced Dependencies](#advanced-dependencies)
  - [Recursion](#recursion)
  - [Package Blacklist](#package-blacklist)
  - [Extra System Dependencies (*apt*)](#extra-system-dependencies-apt)
  - [Extra System Dependencies (*pip*)](#extra-system-dependencies-pip)
  - [Custom Installation Script](#custom-installation-script)
  - [Extra Image Files](#extra-image-files)
- [Additional Information](#additional-information)
  - [User Setup](#user-setup)
  - [Slim Deployment Image](#slim-deployment-image)
- [Configuration Variables](#configuration-variables)

We recommend to use *docker-ros* in combination with our other tools for Docker and ROS.
- [*docker-ros-ml-images*](https://github.com/ika-rwth-aachen/docker-ros-ml-images) provides machine learning-enabled ROS Docker images <a href="https://github.com/ika-rwth-aachen/docker-ros-ml-images"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-ros-ml-images?style=social"/></a>
- [*docker-run*](https://github.com/ika-rwth-aachen/docker-run) is a CLI tool for simplified interaction with Docker images <a href="https://github.com/ika-rwth-aachen/docker-run"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-run?style=social"/></a>


## About

*docker-ros* provides a generic [Dockerfile](docker/Dockerfile) that can be used to build development and deployment Docker images for arbitrary ROS packages or package stacks. Building such images can easily be automated by integrating *docker-ros* into CI through the provided [GitHub action](action.yml) or [GitLab CI template](.gitlab-ci/docker-ros.yml). The development image built by *docker-ros* contains all required dependencies and the source code of your ROS-based repository. The deployment image only contains dependencies and the compiled binaries created by building the ROS packages in the repository. *docker-ros* is also able to build multi-arch Docker images for *amd64* and *arm64* architectures. In addition, [*slim*](https://github.com/slimtoolkit/slim) is integrated for slimming Docker image size of the deployment image by up to 30x (see [*Slim Deployment Image*](#slim-deployment-image)).

The Dockerfile performs the following steps to build these images:
1. All dependency repositories that are defined in a `.repos` file anywhere in the repository are cloned using [*vcstool*](https://github.com/dirk-thomas/vcstool).
2. *(optional)* Packages blacklisted in a special file `blacklisted-packages.txt` are removed from the workspace (see [*Advanced Dependencies*](#package-blacklist)).
3. The ROS dependencies listed in each package's `package.xml` are installed by [*rosdep*](https://docs.ros.org/en/independent/api/rosdep/html/).
4. *(optional)* Additional apt dependencies from a special file `additional-debs.txt` are installed, if needed (see [*Advanced Dependencies*](#extra-system-dependencies-apt)).
5. *(optional)* Additional pip requirements from a special file `additional-pip-requirements.txt` are installed, if needed (see [*Advanced Dependencies*](#extra-system-dependencies-pip)).
6. *(optional)* A special folder `additional-files/` is copied into the images, if needed (see [*Advanced Dependencies*](#extra-image-files)).
7. *(optional)* A special script `custom.sh` is executed to perform further arbitrary installation commands, if needed (see [*Advanced Dependencies*](#custom-installation-script)).
8. *(deployment)* All ROS packages are built using `catkin` (ROS) or `colcon` (ROS2).
9. *(deployment)* A custom launch command is configured to run on container start.

### Prerequisites

*docker-ros* is made for automated execution in GitHub or GitLab CI pipelines. For local execution, see [*Build images locally*](#build-images-locally).

<details><summary>GitHub</summary>

GitHub offers free minutes on GitHub-hosted runners executing GitHub Actions, [see here](https://docs.github.com/en/billing/managing-billing-for-github-actions/about-billing-for-github-actions). No further setup is required other than integrating *docker-ros* into your repository, see [*Usage*](#usage).

Note that GitHub is currently only offering Linux runners based on the *amd64* architecture. *docker-ros* can also build multi-arch Docker images solely on the *amd64* platform through emulation, but performance can be improved greatly by deploying [self-hosted runners](https://docs.github.com/en/actions/hosting-your-own-runners/managing-self-hosted-runners/about-self-hosted-runners) for the *arm64* platform.

</details>

<details><summary>GitLab</summary>

> [!NOTE]  
> - GitLab runners must be based on the Docker executor, [see here](https://docs.gitlab.com/runner/executors/docker.html)
> - GitLab runners must run in privileged mode for Docker-in-Docker, [see here](https://docs.gitlab.com/runner/executors/docker.html#use-docker-in-docker-with-privileged-mode)
> - GitLab runners must be tagged with tags `privileged` and either `amd64` or `arm64` depending on their architecture

GitLab offers free minutes on GitLab-hosted runners executing GitLab CI pipelines on [gitlab.com](https://gitlab.com), [see here](https://docs.gitlab.com/runner/#use-gitlabcom-saas-runners). On self-hosted GitLab instances, you can set up self-hosted runners, [see here](https://docs.gitlab.com/runner/#use-self-managed-runners).

Note that GitLab is currently only offering Linux runners based on the *amd64* architecture. *docker-ros* can also build multi-arch Docker images solely on the *amd64* platform through emulation, but performance can be improved greatly by deploying [self-hosted runners](https://docs.gitlab.com/runner/#use-self-managed-runners) for the *arm64* platform.

</details>


## Usage

*docker-ros* can easily be integrated into any GitHub or GitLab repository containing ROS packages. Example integrations can be found in the following sections. Configuration options can be found [here](#configuration-variables). For local execution, see [*Build images locally*](#build-images-locally).

<details open><summary>GitHub</summary>

*docker-ros* provides a [GitHub action](action.yml) that can simply be added to a job via the [`jobs.<job_id>.steps[*].uses` keyword](https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions#jobsjob_idstepsuses). A quick start for GitHub Actions is found [here](https://docs.github.com/en/actions/quickstart).

</details>

<details open><summary>GitLab</summary>

*docker-ros* provides a [GitLab CI template](.gitlab-ci/docker-ros.yml) that can simply be included in a [`.gitlab-ci.yml`](https://docs.gitlab.com/ee/ci/yaml/gitlab_ci_yaml.html) file. A quick start for GitLab CI is found [here](https://docs.gitlab.com/ee/ci/quick_start/).

</details>

### Build a minimal image for deployment

<details open><summary>GitHub</summary>

```yml
on: push
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@v1.6.2
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
```

</details>

<details open><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/v1.6.2/.gitlab-ci/docker-ros.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
```

</details>

### Build development and deployment images

<details><summary>GitHub</summary>

```yml
on: push
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@v1.6.2
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          target: dev,run
```

</details>

<details><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/v1.6.2/.gitlab-ci/docker-ros.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
  TARGET: dev,run
```

</details>

### Build multi-arch images

<details><summary>GitHub</summary>

```yml
on: push
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@v1.6.2
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          target: dev,run
          platform: amd64,arm64
```

</details>

<details><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/v1.6.2/.gitlab-ci/docker-ros.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
  TARGET: dev,run
  PLATFORM: amd64,arm64
```

</details>

### Build deployment image with additional industrial_ci check

<details><summary>GitHub</summary>

```yml
on: push
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@v1.6.2
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          enable-industrial-ci: 'true'
```

</details>

<details><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/v1.6.2/.gitlab-ci/docker-ros.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
  ENABLE_INDUSTRIAL_CI: 'true'
```

</details>

### Build multi-arch images on arch-specific self-hosted runners in parallel

<details><summary>GitHub</summary>

```yml
on: push
jobs:
  docker-ros:
    strategy:
      matrix:
        target: [dev, run]
        platform: [amd64, arm64]
    runs-on: [self-hosted, "${{ matrix.platform }}"]
    steps:
      - uses: ika-rwth-aachen/docker-ros@v1.6.2
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          target: ${{ matrix.target }}
          platform: ${{ matrix.platform }}
          enable-singlearch-push: true
      # TODO: manifest
```

</details>

<details><summary>GitLab</summary>

```yml
# TODO
```

</details>

### Build images locally

*docker-ros* can build Docker images locally by executing the [`build.sh`](scripts/build.sh) script.

1. Clone *docker-ros* as a Git submodule to `docker/docker-ros` in your repository.
    ```bash
    # ros-repository/
    mkdir -p docker
    git submodule add https://github.com/ika-rwth-aachen/docker-ros.git docker/docker-ros
    ```
2. Configure the build using the same [environment variables](#configuration-variables) as used for GitLab CI and run [`build.sh`](scripts/build.sh), e.g.:
    ```bash
    # ros-repository/
      BASE_IMAGE="rwthika/ros2:humble" \
      COMMAND="ros2 run my_pkg my_node" \
      IMAGE="my-image:latest" \
    ./docker/docker-ros/scripts/build.sh
    ```
    > [!NOTE]  
    > You can alternatively store your environment variable configuration in a `.env` file:
    > ```bash
    > # .env
    > BASE_IMAGE="rwthika/ros2:humble"
    > COMMAND="ros2 run my_pkg my_node"
    > IMAGE="my-image:latest"
    > ```


## Advanced Dependencies

In order to keep things organized, we recommend to place all *docker-ros* related files in a `docker` folder on top repository level.

### Recursion

Most of the steps listed in [*About*](#about) and below can be toggled between recursive and non-recursive mode, see [*Configuration Variables*](#configuration-variables). This usually means that not only special files on the top-level are considered (e.g., `docker/additional-requirements.txt`), but also files with the same name (e.g., `additional-requirements.txt`) that are found anywhere in the workspace, even after cloning the upstream repositories in step 1.

### Package Blacklist

If your ROS-based repository (or any of your repository's upstream dependencies, see `.repos`) contains ROS packages that should neither be built nor be used for determining dependencies, you can blacklist those in a special `blacklisted-packages.txt` file.

Create a file `blacklisted-packages.txt` in your `docker` folder (or configure a different `BLACKLISTED_PACKAGES_FILE`) and list any ROS package name to blacklist.

### Extra System Dependencies (*apt*)

If your ROS-based repository requires system dependencies that cannot be installed by specifying their [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/) keys in a `package.xml`, you can use a special `additional-debs.txt` file.

Create a file `additional-debs.txt` in your `docker` folder (or configure a different `ADDITIONAL_DEBS_FILE`) and list any other dependencies that need to be installed via *apt*.

### Extra System Dependencies (*pip*)

If your ROS-based repository requires Python dependencies that cannot be installed by specifying their [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/) keys in a `package.xml`, you can use a special `additional-pip-requirements.txt` file.

Create a file `additional-pip-requirements.txt` in your `docker` folder (or configure a different `ADDITIONAL_PIP_FILE`) and list any other Python dependencies that need to be installed via *pip*.

### Custom Installation Script

If your ROS-based repository requires to execute any other installation or pre-/post-installation steps, you can use a special `custom.sh` script.

Create a script `custom.sh` in your `docker` folder (or configure a different `CUSTOM_SCRIPT_FILE`) that executes arbitrary commands as part of the image building process.

### Extra Image Files

If you need to have additional files present in the deployment image, you can use a special `additional-files` folder. The folder contents will be copied into the container before the custom installation script `custom.sh` is executed.

Create a folder `additional-files` in your `docker` folder (or configure a different `ADDITIONAL_FILES_DIR`) and place any files or directories in it. The contents will be copied to `/docker-ros/additional-files` in the image.


## Additional Information

### User Setup

Containers of the provided images start with `root` user by default. If the two environment variables `DOCKER_UID` and `DOCKER_GID` are passed, a new user with the corresponding UID/GID is created on the fly. Most importantly, this features allows to mount and edit files of the host user in the container without having to deal with permission issues.

```bash
docker run --rm -it -e DOCKER_UID=$(id -u) -e DOCKER_GID=$(id -g) -e DOCKER_USER=$(id -un) rwthika/ros:latest
```

The password of the custom user is set to its username (`dockeruser:dockeruser` by default).

### Slim Deployment Image

*docker-ros* integrates the [*slim*](https://github.com/slimtoolkit/slim) toolkit for minifying container images. *slim* is enabled by default and will, in addition to the `run` deployment image, produce an additional `:latest-slim`-tagged minified image. Note that the slimmed deployment image is stripped of every single thing not needed for executing the default launch command. The slimming process can be controlled via the `SLIM_BUILD_ARGS` configuration variable.


## Configuration Variables

> [!NOTE]  
> *GitHub Action input* | *GitLab CI environment variable*

- **`additional-debs-file` | `ADDITIONAL_DEBS_FILE`**  
  Relative filepath to file containing additional apt deb packages to install  
  *default:* `docker/additional-debs.txt`  
- **`additional-files-dir` | `ADDITIONAL_FILES_DIR`**  
  Relative path to directory containing additional files to copy into image  
  *default:* `docker/additional-files`  
- **`additional-pip-file` | `ADDITIONAL_PIP_FILE`**  
  Relative filepath to file containing additional pip packages to install  
  *default:* `docker/additional-pip-requirements.txt`
- **`base-image` | `BASE_IMAGE`**  
  Base image `name:tag`  
  *required*  
- **`blacklisted-packages-file` | `BLACKLISTED_PACKAGES_FILE`**  
  Relative filepath to file containing blacklisted packages  
  *default:* `docker/blacklisted-packages.txt`
- **`build-context` | `BUILD_CONTEXT`**  
  Build context of Docker build process  
  *default:* `${{ github.workspace }}` | `.`  
- **`command` | `COMMAND`**  
  Launch command of run image  
  *required if `target=run`*  
- **`custom-script-file` | `CUSTOM_SCRIPT_FILE`**  
  Relative filepath to script containing custom installation commands  
  *default:* `docker/custom.sh`  
- **`dev-image-name` | `DEV_IMAGE_NAME`**  
  Image name of dev image  
  *default:* `<IMAGE_NAME>`  
- **`dev-image-tag` | `DEV_IMAGE_TAG`**  
  Image tag of dev image  
  *default:* `<IMAGE_TAG>-dev`  
- **`disable-ros-installation` | `DISABLE_ROS_INSTALLATION`**  
  Disable automatic installation of `ros-$ROS_DISTRO-ros-core` package  
  *e.g., if ROS is already installed in `base-image` and package is not available for the OS*  
  *default:* `false`
- **`-` | `DOCKER_ROS_GIT_REF`**  
  Git ref of *docker-ros* to run in CI  
  *default:* `main` 
- **`enable-checkout` | `-`**  
  Enable [*checkout*](https://github.com/actions/checkout) action to (re-)download your repository prior to running the pipeline  
  *default:* `true`
- **`enable-checkout-submodules` | `-`**  
  Enable submodules for the [*checkout*](https://github.com/actions/checkout) action (`false`|`true`|`recursive`)  
  *default:* `recursive`
- **`enable-checkout-lfs` | `-`**  
  Enable [*Git LFS*](https://git-lfs.com/) support for the [*checkout*](https://github.com/actions/checkout) action  
  *default:* `true` 
- **`enable-industrial-ci` | `ENABLE_INDUSTRIAL_CI`**  
  Enable [*industrial_ci*](https://github.com/ros-industrial/industrial_ci)  
  *default:* `false` 
- **`enable-push-as-latest` | `ENABLE_PUSH_AS_LATEST`**  
  Push images with tag `latest`/`latest-dev` in addition to the configured image names  
  *default:* `false`  
- **`enable-singlearch-push` | `ENABLE_SINGLEARCH_PUSH`**  
  Enable push of single arch images with `-amd64`/`-arm64` postfix  
  *default:* `false` 
- **`enable-recursive-additional-debs` | `ENABLE_RECURSIVE_ADDITIONAL_DEBS`**  
  Enable recursive discovery of files named `additional-debs-file`  
  *default:* `false`
- **`enable-recursive-additional-pip` | `ENABLE_RECURSIVE_ADDITIONAL_PIP`**  
  Enable recursive discovery of files named `additional-pip-file`  
  *default:* `false`
- **`enable-recursive-blacklisted-packages` | `ENABLE_RECURSIVE_BLACKLISTED_PACKAGES`**  
  Enable recursive discovery of files named `blacklisted-packages-file`  
  *default:* `false`
- **`enable-recursive-custom-script` | `ENABLE_RECURSIVE_CUSTOM_SCRIPT`**  
  Enable recursive discovery of files named `custom-script-file`  
  *default:* `false`
- **`enable-recursive-vcs-import` | `ENABLE_RECURSIVE_VCS_IMPORT`**  
  Enable recursive discovery of files named `*.repos`  
  *default:* `true`
- **`enable-slim` | `ENABLE_SLIM`**  
  Enable an extra slimmed run image via [slim](https://github.com/slimtoolkit/slim) (only if `run` stage is targeted)  
  *default:* `true`
- **`git-https-password` | `GIT_HTTPS_PASSWORD`**  
  Password for cloning private Git repositories via HTTPS  
  *default:* `${{ github.token }}` | `$CI_JOB_TOKEN` 
- **`git-https-server` | `GIT_HTTPS_SERVER`**  
  Server URL (without protocol) for cloning private Git repositories via HTTPS  
  *default:* `github.com` | `$CI_SERVER_HOST:$CI_SERVER_PORT` 
- **`git-https-user` | `GIT_HTTPS_USER`**  
  Username for cloning private Git repositories via HTTPS  
  *default:* `${{ github.actor }}` | `gitlab-ci-token`  
- **`git-ssh-known-host-keys` | `GIT_SSH_KNOWN_HOST_KEYS`**  
  Known SSH host keys for cloning private Git repositories via SSH (may be obtained using `ssh-keyscan`)  
- **`git-ssh-private-key` | `GIT_SSH_PRIVATE_KEY`**  
  SSH private key for cloning private Git repositories via SSH  
- **`image-name` | `IMAGE_NAME`**  
  Image name of run image  
  *default:* `ghcr.io/${{ github.repository }}` | `$CI_REGISTRY_IMAGE`  
- **`image-tag` | `IMAGE_TAG`**  
  Image tag of run image
  *default:* `latest`  
- **`platform` | `PLATFORM`**  
  Target platform architecture (comma-separated list)  
  *default:* runner architecture | `amd64`
  *supported values:* `amd64`, `arm64`
- **`registry` | `REGISTRY`**  
  Docker registry to push images to  
  *default:* `ghcr.io` | `$CI_REGISTRY`  
- **`registry-password` | `REGISTRY_PASSWORD`**  
  Docker registry password  
  *default:* `${{ github.token }}` | `$CI_REGISTRY_PASSWORD`  
- **`registry-user` | `REGISTRY_USER`**  
  Docker registry username  
  *default:* `${{ github.actor }}` | `$CI_REGISTRY_USER`  
- **`rmw-implementation` | `RMW_IMPLEMENTATION`**  
  ROS 2 middleware implementation  
  *default:* `rmw_cyclonedds_cpp`  
  *supported values:* `rmw_zenoh_cpp`, `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, `rmw_gurumdds_cpp`, ...  
- **`rmw-zenoh-git-ref` | `RMW_ZENOH_GIT_REF`**  
  Git ref of rmw_zenoh repo to build if `RMW_IMPLEMENTATION=rmw_zenoh_cpp`  
  *default:* `$ROS_DISTRO`  
- **`ros-distro` | `ROS_DISTRO`**  
  ROS Distro  
  *required if ROS is not installed in `base-image`*  
  *supported values:* `rolling`, ..., `noetic`, ...
- **`slim-build-args` | `SLIM_BUILD_ARGS`**  
  [Arguments to `slim build`](https://github.com/slimtoolkit/slim?tab=readme-ov-file#build-command-options) (except for `--target` and `--tag`)  
  *default:* `--sensor-ipc-mode proxy --continue-after=10 --show-clogs --http-probe=false`  
- **`slim-image-name` | `SLIM_IMAGE_NAME`**  
  Image name of slim run image  
  *default:* `<IMAGE_NAME>`  
- **`slim-image-tag` | `SLIM_IMAGE_TAG`**  
  Image tag of slim run image  
  *default:* `<IMAGE_TAG>-slim`  
- **`target` | `TARGET`**  
  Target stage of Dockerfile (comma-separated list)  
  *default:* `run`
  *supported values:* `dev`, `run`
- **`vcs-import-file` | `VCS_IMPORT_FILE`**  
  Relative filepath to file containing additional repos to install via vcstools (only relevant if `enable-recursive-vcs-import=false`)  
  *default:* `.repos`
