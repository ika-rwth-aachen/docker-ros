# docker-ros

`docker-ros` automatically builds development and deployment Docker images for your ROS-based repositories.

## Integration

1. Add a `docker` folder to your repository and clone `docker-ros` there as a submodule.
    ```bash
    # ros-repository/
    mkdir -p docker
    git submodule add https://gitlab.ika.rwth-aachen.de/ops/docker-ros.git
    cd docker
    ```
1. Copy the template [`docker-compose.yaml`](docker-compose.yaml) to your `docker` folder.
    ```bash
    # ros-repository/docker/
    cp docker-ros/docker-compose.yaml .
    ```
1. Edit the copied `docker-compose.yaml` to specify key information about the images built for your repository. Note that only the top section of the file requires changes.
    - `x-base-image: &base-image`: base image for the images to be built; assumes that ROS/ROS2 is installed; it is suggested to choose the most minimal [of our custom ROS images](https://gitlab.ika.rwth-aachen.de/automated-driving/docker#available-images):
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros1:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros1-cuda:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros1-tf:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros1-torch:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros1-ml:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros2:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros2-cuda:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros2-tf:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros2-torch:latest`
      - `gitlab.ika.rwth-aachen.de:5050/automated-driving/docker/ros2-ml:latest`
    - `x-dev-image: &dev-image`: image name and tag of the development image to be built; it is suggested to use `gitlab.ika.rwth-aachen.de:5050/<GROUP>/<REPOSITORy>:latest-dev`
    - `x-run-image: &run-image`: image name and tag of the deployment image to be built; it is suggested to use `gitlab.ika.rwth-aachen.de:5050/<GROUP>/<REPOSITORy>:latest`
    - `x-command: &command`: default Dockerfile [`CMD`](https://docs.docker.com/engine/reference/builder/#cmd) command of the deployment image
1. Create a new `.gitlab-ci.yml` file on the top level of your repository with the following contents. It will automatically include the pre-defined [`.gitlab-ci.template.yml`](.gitlab-ci.template.yml).
    ```bash
    # ros-repository/
    include:
      - project: ops/docker-ros
        ref: main
        file: .gitlab-ci.template.yml
    ```
