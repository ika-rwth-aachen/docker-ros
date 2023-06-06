# *docker-ros* – Automated Containerization of ROS Applications

<p align="center">
  <img src="https://img.shields.io/github/v/release/ika-rwth-aachen/docker-ros"/></a>
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/docker-ros"/>
  <a href="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/github.yml"><img src="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/github.yml/badge.svg"/></a>
  <a href="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/gitlab.yml"><img src="https://github.com/ika-rwth-aachen/docker-ros/actions/workflows/gitlab.yml/badge.svg"/></a>
</p>

*docker-ros* automatically builds minimal container images of ROS applications.

- [Usage](#usage)
  - [Build a minimal image for deployment](#build-a-minimal-image-for-deployment)
  - [Build development and deployment images](#build-development-and-deployment-images)
  - [Build multi-arch images](#build-multi-arch-images)
  - [Build deployment image with additional industrial\_ci check](#build-deployment-image-with-additional-industrial_ci-check)
- [Configuration Variables](#configuration-variables)

We recommend to use *docker-ros* in combination with our other tools for Docker and ROS.
- [*docker-ros-ml-images*](https://github.com/ika-rwth-aachen/docker-ros-ml-images) provides machine learning-enabled ROS Docker images <a href="https://github.com/ika-rwth-aachen/docker-ros-ml-images"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-ros-ml-images?style=social"/></a>
- [*docker-run*](https://github.com/ika-rwth-aachen/docker-run) is a CLI tool for simplified interaction with Docker images <a href="https://github.com/ika-rwth-aachen/docker-run"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-run?style=social"/></a>


## About

...


## Usage

### Build a minimal image for deployment

<details open><summary>GitHub</summary>

```yml
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@main
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
```

</details>

<details open><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/main/templates/.gitlab-ci.template.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
```

</details>

### Build development and deployment images

<details><summary>GitHub</summary>

```yml
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@main
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          target: dev,run
```

</details>

<details><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/main/templates/.gitlab-ci.template.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
  TARGET: dev,run
```

</details>

### Build multi-arch images

<details><summary>GitHub</summary>

```yml
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@main
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
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/main/templates/.gitlab-ci.template.yml

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
jobs:
  docker-ros:
    runs-on: ubuntu-latest
    steps:
      - uses: ika-rwth-aachen/docker-ros@main
        with:
          base-image: rwthika/ros2:humble
          command: ros2 run my_pkg my_node
          enable-industrial-ci: 'true'
```

</details>

<details><summary>GitLab</summary>

```yml
include:
  - remote: https://raw.githubusercontent.com/ika-rwth-aachen/docker-ros/main/templates/.gitlab-ci.template.yml

variables:
  BASE_IMAGE: rwthika/ros2:humble
  COMMAND: ros2 run my_pkg my_node
  ENABLE_INDUSTRIAL_CI: 'true'
```

</details>

### Build multi-arch images on arch-specific self-hosted runners in parallel

<details><summary>GitHub</summary>

```yml
jobs:
  docker-ros:
    strategy:
      matrix:
        target: [dev, run]
        platform: [amd64, arm64]  
    runs-on: [self-hosted, "${{ matrix.platform }}"]
    steps:
      - uses: ika-rwth-aachen/docker-ros@main
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

## Configuration Variables

| GitHub Input | GitLab Env Var | Required | Description | Default GitHub | Default GitLab | Allow. Values |
| --- | --- | :---: | --- | :---: | :---: | :---: |
| `base_image` | `BASE_IMAGE` | x | Base image `name:tag` | - | - | - |
| `build-context` | `BUILD_CONTEXT` | - | Build context of Docker build process | `${{ github.workspace }}` | `.` | - |
| `command` | `COMMAND` | - | Launch command of run image (required if `target=run`) | - | - | - |
| `dev-image-name` | `DEV_IMAGE_NAME` | - | Image name of dev image |  |  | - |
| `dev-image-tag` | `DEV_IMAGE_TAG` | - | Image tag of dev image | `<IMAGE_NAME>` | `"<IMAGE_TAG>-dev` | - |
| - | `DOCKER_ROS_GIT_REF` | - | Git reference of *docker-ros* to run in CI | - | `main` | - |
| `enable-industrial-ci` | `ENABLE_INDUSTRIAL_CI` | - | Enable industrial_ci | `false` | `false` | `true`, `false` |
| `enable-singlearch-push` | `ENABLE_SINGLEARCH_PUSH` | - | Enable push of single arch images with `-amd64`/`-arm64` postfix | `false` | `false` | `true`, `false` |
| `git-https-password` | `GIT_HTTPS_PASSWORD` | - | Password for cloning private Git repositories via HTTPS | `${{ github.token }}` | `$CI_JOB_TOKEN` | - |
| `git-https-server` | `GIT_HTTPS_SERVER` | - | Server URL (without protocol) for cloning private Git repositories via HTTPS | `github.com` | `$CI_SERVER_HOST:$CI_SERVER_PORT` | - |
| `git-https-user` | `GIT_HTTPS_USER` | - | Username for cloning private Git repositories via HTTPS | `${{ github.actor }}` | `gitlab-ci-token` |  |
| `git-ssh-known-host-keys` | `GIT_SSH_KNOWN_HOST_KEYS` | - | Known SSH host keys for cloning private Git repositories via SSH (may be obtained using `ssh-keyscan`) | - | - | - |
| `git-ssh-private-key` | `GIT_SSH_PRIVATE_KEY` | - | SSH private key for cloning private Git repositories via SSH | - | - | - |
| `image-name` | `IMAGE_NAME` | - | Image name of run image | `ghcr.io/${{ github.repository }}` | `$CI_REGISTRY_IMAGE` | - |
| `image-tag` | `IMAGE_TAG` | - | Image tag of run image | `latest` | `latest` | - |
| `platform` | `PLATFORM` | - | Target platform architecture (comma-separated list) | runner architecture | runner architecture | `amd64`, `arm64` |
| `registry-password` | `REGISTRY_PASSWORD` | - | Docker registry password | `${{ github.token }}` | `$CI_REGISTRY_PASSWORD` | - |
| `registry-username` | `REGISTRY_USERNAME` | - | Docker registry username | `${{ github.actor }}` | `$CI_REGISTRY_USER` | - |
| `registry` | `REGISTRY` | - | Docker registry to push images to | `ghcr.io` | `$CI_REGISTRY` | - |
| `target` | `TARGET` | - | Target stage of Dockerfile (comma-separated list) | `run` | `run` | `dev`, `run`, `dev,run` |
