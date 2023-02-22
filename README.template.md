# zed-camera

ROS driver for [Stereolabs ZED stereo cameras](https://www.stereolabs.com/)

- [Nodes](#nodes)
  - [`zed_wrapper/zed_wrapper`](#zed_wrapperzed_wrapper)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Default Command](#default-command)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)
  - [Additional Remarks](#additional-remarks)
- [Official Documentation](#official-documentation)

---

## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `zed_wrapper` | `zed_wrapper` | main ZED driver node |

### `zed_wrapper/zed_wrapper`

#### Subscribed Topics

\-

#### Published Topics

[See documentation](https://www.stereolabs.com/docs/ros2/zed-node/#published-topics)

// TODO: copy here or only link?
| Topic | Type | Description |
| --- | --- | --- |
| `~/left/camera_info` | `sensor_msgs/msg/CameraInfo` | left camera calibration data |
| `~/left/image_rect_color` | `sensor_msgs/msg/Image` | left camera color rectified image |
| `~/left/image_rect_gray` | `sensor_msgs/msg/Image` | left camera gray rectified image |
| `~/left_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | left camera raw calibration data |
| `~/left_raw/image_rect_color` | `sensor_msgs/msg/Image` | left camera color unrectified image |
| `~/left_raw/image_rect_gray` | `sensor_msgs/msg/Image` | left camera gray unrectified image |
| `~/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | left camera calibration data |
| `~/rgb/image_rect_color` | `sensor_msgs/msg/Image` | left camera color rectified image |
| `~/rgb/image_rect_gray` | `sensor_msgs/msg/Image` | left camera gray rectified image |
| `~/rgb_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | left camera raw calibration data |
| `~/rgb_raw/image_rect_color` | `sensor_msgs/msg/Image` | left camera color unrectified image |
| `~/rgb_raw/image_rect_gray` | `sensor_msgs/msg/Image` | left camera gray unrectified image |
| `~/right/camera_info` | `sensor_msgs/msg/CameraInfo` | right camera calibration data |
| `~/right/image_rect_color` | `sensor_msgs/msg/Image` | right camera color rectified image |
| `~/right/image_rect_gray` | `sensor_msgs/msg/Image` | right camera gray rectified image |
| `~/right_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | right camera raw calibration data |
| `~/right_raw/image_rect_color` | `sensor_msgs/msg/Image` | right camera color unrectified image |
| `~/right_raw/image_rect_gray` | `sensor_msgs/msg/Image` | right camera gray unrectified image |
| `~/stereo/image_rect_color` | `sensor_msgs/msg/Image` | side-by-side left/right rectified stereo pair |
| `~/stereo/image_raw_color` | `sensor_msgs/msg/Image` | side-by-side left/right unrectififed stereo pair |

#### Services

[See documentation](https://www.stereolabs.com/docs/ros2/zed-node/#services)

#### Actions

\-

#### Parameters

[See documentation](https://www.stereolabs.com/docs/ros2/zed-node/#configuration-parameters)

---

## Usage of docker-ros Images

### Default Command

```bash
ros2 launch zed_wrapper zed2i.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `zed_wrapper` | `zed.launch.py` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/launch/` | driver for ZED |
| `zed_wrapper` | `zed2.launch.py` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/launch/` | driver for ZED 2 |
| `zed_wrapper` | `zed2i.launch.py` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/launch/` | driver for ZED 2i |
| `zed_wrapper` | `zedm.launch.py` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/launch/` | driver for ZED Mini |

### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `zed_wrapper` | `common.yaml` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/config/` | general, video, depth, tracking, mapping, sensor, detection, debug settings |
| `zed_wrapper` | `zed.yaml` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/config/` | camera name and min/max depth for ZED |
| `zed_wrapper` | `zed2.yaml` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/config/` | camera name and min/max depth for ZED 2 |
| `zed_wrapper` | `zed2i.yaml` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/config/` | camera name and min/max depth for ZED 2i |
| `zed_wrapper` | `zedm.yaml` | `/docker-ros/ws/install/zed_wrapper/share/zed_wrapper/config/` | camera name and min/max depth for ZED Mini |

### Additional Remarks

\-

---

## Official Documentation

- [Published topics](https://www.stereolabs.com/docs/ros2/zed-node/#published-topics)
- [Configuration parameters](https://www.stereolabs.com/docs/ros2/zed-node/#configuration-parameters)
- [Dynamic parameters](https://www.stereolabs.com/docs/ros2/zed-node/#dynamic-parameters)
- [Transform frame](https://www.stereolabs.com/docs/ros2/zed-node/#transform-frame)
- [Services](https://www.stereolabs.com/docs/ros2/zed-node/#services)
