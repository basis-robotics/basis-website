---
layout: post
title: Basis Robot 02 - Software
author: Kyle Franz
---

I've spent the past few weeks working on a small robot to both be able to give demos with and exercise our code.

- [Part 01 - Hardware]({% post_url 2024-11-22-basis-robot-01-hardware %})
- [Part 02 - Software]({% post_url 2024-11-24-basis-robot-02-software %}) (You're here!)
- Part 03 - tf2 support and LiDAR 

# The Software:
{% mermaid %}
---
config:
  flowchart:
    nodeSpacing: 5
    subGraphTitleMargin:
      bottom: 10
  defaultRenderer: elk
  elk:
      mergeEdges: True
---
graph LR
  %%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
  subgraph unit_/foxglove/foxglove["/foxglove/foxglove"]
    handler_/foxglove/foxglove:::hidden
  end

  subgraph unit_/webcam/yuyv_to_rgb["/webcam/yuyv_to_rgb"]
    handler_/webcam/yuyv_to_rgb::OnYUYV[["OnYUYV()"]]
  end

  subgraph unit_/webcam/v4l2_camera_driver["/webcam/v4l2_camera_driver"]
    handler_/webcam/v4l2_camera_driver::OnCameraImage[["OnCameraImage()"]]
  end
handler_/webcam/v4l2_camera_driver::OnCameraImage --/camera/yuyv--> handler_/webcam/yuyv_to_rgb::OnYUYV
handler_/webcam/yuyv_to_rgb::OnYUYV --/camera/rgb--x handler_/webcam/yuyv_to_rgb::OnYUYV::dummy:::hidden

{% endmermaid %}
