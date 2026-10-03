# Camera and voice vision

The Logitech Brio 100 is a USB camera with an integrated microphone.

## Camera topics and Foxglove

`src/senses/launch/eyes.launch.py` starts the external `v4l2_camera` driver and
`topic_tools` throttle. The removed custom `eyes` node only logged startup; it
performed no capture or processing.

The retained camera path is:

```text
v4l2_camera -> /image_raw/compressed
           -> image_throttle -> /image_viz/compressed
```

Compressed images require the image-transport plugins in the ROS environment.
The launch defaults are:

| Argument | Default |
| --- | --- |
| `image_in` | `/image_raw/compressed` |
| `image_viz` | `/image_viz/compressed` |
| `image_viz_rate_hz` | `3.0` |

The throttle limits visualization traffic. The Foxglove bridge exposes
`/image_viz/compressed` and camera-info topics through its topic whitelist.
Bridge settings belong to `src/bringup/launch/all.launch.py`; use that file as
the current configuration rather than copying an older tuning example.

## Current voice vision

The Nova voice agent subscribes to the latest `sensor_msgs/msg/CompressedImage`
on `/image_viz/compressed` and sends still-frame questions to Bedrock Nova Lite.
Its default model is `amazon.nova-lite-v1:0`; AWS profile and region are shared
with the voice agent. Example questions include "What can you see?" and
"What am I holding?". This is still-frame Q&A rather than continuous analysis.

The [VENTUNO migration](VENTUNO_MIGRATION.md) will add GenieX vision for the local
backend while retaining Nova as an option. Camera device configuration and ROS
packages will be included in the container deployment. Hardware and Mac/Foxglove
checks remain pending; see [installation status](INSTALL.md).
