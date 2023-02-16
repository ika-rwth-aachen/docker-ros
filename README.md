<img src="assets/logo.png" height=130 align="right">

# docker-ros

*docker-ros* automatically builds development and deployment Docker images for your ROS-based repositories.

## Features

*docker-ros* provides a generic [Dockerfile](Dockerfile) that can be used to build development and deployment Docker images for arbitrary ROS packages or package stacks. It also provides a [GitLab CI configuration](.gitlab-ci.template.yml) that automatically builds these Docker images. The development image contains all required dependencies and the source code of your ROS-based repository. The deployment image only contains dependencies and the compiled binaries created by building the ROS packages in the repository.

The Dockerfile performs the following steps to automatically build these images:
1. All dependency repositories that are defined in a `.repos` file anywhere in the repository are cloned using [vcstool](https://github.com/dirk-thomas/vcstool).
1. The ROS dependencies listed in each package's `package.xml` are installed by [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/).
1. *(optional)* Additional dependencies from a special file `additional.apt-dependencies` are installed, if needed.
1. *(optional)* A special folder `files/` is copied into the images, if needed.
1. *(optional)* A special script `custom.sh` is executed to perform further arbitrary installation commands, if needed.
1. *(deployment)* All ROS packages are built using `catkin` (ROS) or `colcon` (ROS2). 
1. *(deployment)* A custom launch command is configured to run on container start.

## Integration

1. Add a `docker` folder to your repository and clone *docker-ros* there as a submodule.
    ```bash
    # ros-repository/
    mkdir -p docker
    git submodule add <../RELATIVE/PATH/..>/ops/docker-ros.git docker/docker-ros
    ```
1. Copy the template [`docker-compose.yaml`](docker-compose.yaml) to your `docker` folder.
    ```bash
    # ros-repository/docker/
    cp docker-ros/docker-compose.yaml .
    ```
1. Edit the copied `docker-compose.yaml` to specify key information about the images built for your repository. Note that only the top section of the file requires changes.
    - `x-base-image: &base-image`
      - base image for the images to be built
      - assumes that ROS/ROS2 is installed
      - it is suggested to choose the most minimal [of our custom ROS images](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-base#available-images)
    - `x-dev-image: &dev-image`
      - image name and tag of the development image to be built
      - it is suggested to use `gitlab.ika.rwth-aachen.de:5050/<GROUP>/<REPOSITORy>:latest-dev`
    - `x-run-image: &run-image`
      - image name and tag of the deployment image to be built
      - it is suggested to use `gitlab.ika.rwth-aachen.de:5050/<GROUP>/<REPOSITORy>:latest`
    - `x-command: &command`
      - default Dockerfile [`CMD`](https://docs.docker.com/engine/reference/builder/#cmd) command of the deployment image
1. Create a new `.gitlab-ci.yml` file on the top level of your repository with the following contents. It will automatically include the pre-defined [`.gitlab-ci.template.yml`](.gitlab-ci.template.yml).
    ```yaml
    include:
      - project: fb-fi/ops/docker-ros
        ref: main
        file: .gitlab-ci.template.yml
    ```
1. In your GitLab project, go to *Settings / General / Visibility, project features, permissions* and enable the *Container registry* to store the automatically built Docker images. Then go to *Settings / Packages and registries / Edit cleanup rules* and configure an image cleanup rule to *Remove tags matching* `.*_ci-.*`.
1. Build the images locally using [`docker compose`](https://docs.docker.com/compose/) from the `docker` folder or push the changes to your repository to have the GitLab CI pipeline build the images automatically.
    ```bash
    # ros-repository/docker/
    docker compose build dev run
    ```

### Advanced Integration

#### System Dependencies (apt)

If your ROS-based repository requires system dependencies that cannot be installed by specifying their [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/) keys in a `package.xml`, you can use the special `additional.apt-dependencies` file.

Create a file `additional.apt-dependencies` in your `docker` folder and list any other dependencies that need to be installed via apt.

#### Custom Installation Script

If your ROS-based repository requires to execute any other installation or pre-/post-installation steps, you can use the special `custom.sh` script.

Create a script `custom.sh` in your `docker` folder that executes arbitrary commands as part of the image building process.

#### Extra Image Files

If you need to have additional files present in the deployment image, you can use the special `files` folder. These will be copied into the container before the custom installation script `custom.sh` is executed.

Create a folder `files` in your `docker` folder and place any files or directories in it. The contents will be copied to `/docker-ros/files` in the image.

### Git Credentials when Building Images Locally

As part of the image build process, all dependency repositories that are defined in a `.repos` file anywhere in the repository are cloned using [vcstool](https://github.com/dirk-thomas/vcstool). This might fail due to missing Git credentials. You can pass Git credentials to build process by creating a special `.env` file.

Create a file `.env` in your `docker` folder and specify username and password. If using GitLab, do not use your personal access credentials, but rather [create a temporary access token](https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html#create-a-personal-access-token).
```
GIT_HTTPS_USER="<TOKEN_NAME>"
GIT_HTTPS_PASSWORD="<TOKEN_PASSWORD>"
```

#### GitLab CI Customization

If needed, you can overwrite any of the [GitLab CI variables of the template CI configuration](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-ros/-/blob/main/.gitlab-ci.template.yml#L14) from your own `.gitlab-ci.yml`.

You can for example disable the [ROS Industrial CI](https://github.com/ros-industrial/industrial_ci) stage with a variable.
```yaml
include:
  - project: ops/docker-ros
    ref: main
    file: .gitlab-ci.template.yml
variables:
  DISABLE_INDUSTRIAL_CI: 'true'
```

| Variable | Description | Default |
| --- | --- | --- |
| `DISABLE_ARCH_AMD64` | toggle the build of amd64 images | `'false'` |
| `DISABLE_ARCH_ARM64` | toggle the build of arm64 images | `'false'` |
| `DISABLE_INDUSTRIAL_CI` | toggle the ROS Industrial CI test job | `'false'` |
