---
layout: post
title: Building a Robot on Basis 02 - Software
author: Kyle Franz
---

I've spent the past few weeks working on a small robot to both be able to give demos with and exercise our code. This post is about the current software architecture for the robot.

- [Part 01 - Hardware]({% post_url 2024-11-22-basis-robot-01-hardware %})
- [Part 02 - Software]({% post_url 2024-11-24-basis-robot-02-software %}) (You're here!)
- Part 03 - tf2 support and LiDAR 

After getting the hardware working, I moved on to the software. This required [a few small changes to the core framework](https://github.com/basis-robotics/basis/pull/64/files) (mostly fixing CMake technicalities), but nothing crazy.

<iframe width="560" height="315" src="https://www.youtube.com/embed/v8CYzripJm0?si=F6h22o1RYu7xqred" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

I can now move the robot around with an wireless controller! The left stick and bumpers control the wheels and the right stick controls the servos.

# The Architecture:
<pre class="mermaid">
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
  subgraph unit_/freenove/rpi_freenove_mecanum_driver["/freenove/rpi_freenove_mecanum_driver"]
    handler_/freenove/rpi_freenove_mecanum_driver::Update[["Update()
100Hz"]]
  end

  subgraph unit_/freenove/rpi_freenove_servo_driver["/freenove/rpi_freenove_servo_driver"]
    handler_/freenove/rpi_freenove_servo_driver::Update[["Update()
100Hz"]]
  end

  subgraph unit_/freenove/rpi_libcamera_driver["/freenove/rpi_libcamera_driver"]
    handler_/freenove/rpi_libcamera_driver::OnCameraImage[["OnCameraImage()
30Hz"]]
  end

  subgraph unit_/freenove/joystick_driver["/freenove/joystick_driver"]
    handler_/freenove/joystick_driver::Tick[["Tick()
20Hz"]]
  end

  subgraph unit_/foxglove/foxglove["/foxglove/foxglove"]
    handler_/foxglove/foxglove:::hidden
  end
handler_/freenove/rpi_freenove_servo_driver::Update::/servo/1/request_degrees:::hidden x--/servo/1/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/rpi_freenove_servo_driver::Update::/servo/0/request_degrees:::hidden x--/servo/0/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/joystick_driver::Tick --/user_inputs--> handler_/freenove/rpi_freenove_mecanum_driver::Update
handler_/freenove/joystick_driver::Tick --/user_inputs--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/rpi_libcamera_driver::OnCameraImage --/camera/rgb--x /camera/rgb::handler_/freenove/rpi_libcamera_driver::OnCameraImage:::hidden
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/1/current_degrees--x /servo/1/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/0/current_degrees--x /servo/0/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
handler_/freenove/rpi_freenove_mecanum_driver::Update --/motor_state--x /motor_state::handler_/freenove/rpi_freenove_mecanum_driver::Update:::hidden
</pre>

This is a pretty straightforward architecture, for now. We run joystick input, allowing it to control both the servos the camera is mounted on as well as the wheels. Later, we'll move the joystick input to the wheels to instead be an input to some sort of planning stick.

This graph was generated with `basis launch --mermaid` - it does a dry run, outputting information about the launch in [mermaid](https://mermaid.js.org/). This is really useful - I can copy/paste directly into a blog post or github markdown document. The PR for this will be merged soon.

# The code

## rpi_libcamera_driver

{% highlight yaml %}{% raw %}
args:
  device:
    type: string
    help: The device to capture from
    optional: True
  
  width:
    type: uint32_t
    default: 1280
  height:
    type: uint32_t
    default: 720
  topic_namespace:
    type: string
    default: /camera

threading_model:
  single
cpp_includes:
  - foxglove/RawImage.pb.h
  - image_conversion.h

handlers:
  OnCameraImage:
    sync:
      # All inputs are required (of which there are technically none)
      type: all
      # Run at 30fps
      rate: 0.03333333333
    outputs:
      {{args.topic_namespace}}/rgb:
        type: protobuf:foxglove.RawImage
        inproc_type: image_conversion::Image
{% endraw %}{% endhighlight %}

This unit implements [libcamera](https://libcamera.org/) support. libcamera on raspberry pi does have a v4l2 interface, I sadly wasn't able to get it working. There's nothing special about this code, [you can see it here](https://github.com/basis-robotics/basis_test_robot/tree/main/unit/rpi_libcamera_driver). It's based off of [the libcamera tutorial in their docs](https://libcamera.org/guides/application-developer.html). The only odd point was that `libcamera::formats::RGB888` appears to be `BGR` - I didn't bother to track down why, I instead just swapped to requesting `BGR`.

Note: it's not perfect that we run this unit at a fixed 30hz - ideally, this unit (and other driver-like units) can update freely on a thread, and publish at will. This requires a change to Basis (`sync: type: external`), which will be made in the next month.

## joystick_driver
{% highlight yaml %}{% raw %}
threading_model:
  single
cpp_includes:
  - basis_robot_input.pb.h
handlers:
  Tick:
    sync:
      type: all
      rate: 0.05
    outputs:
      /user_inputs:
        type: protobuf:basis::robot::input::InputState
{% endraw %}{% endhighlight %}

Again, very straightforward. I initially implemented this using `ioctl` and then switched to [`libevdev`](https://www.freedesktop.org/wiki/Software/libevdev/). This really simplified controller access. [See here.](https://github.com/basis-robotics/basis_test_robot/tree/main/unit/joystick_driver)


## rpi_freenove_servo_driver
{% highlight yaml %}{% raw %}
args:
  i2c_device:
    type: string
    default: "/dev/i2c-1"
  address:
    type: int32_t
    default: 0x40
  default_angle_0:
    type: float
    help: the angle to start at
    default: 0
  default_angle_1:
    type: float
    help: the angle to start at
    default: 0
threading_model:
  single
cpp_includes: 
  - google/protobuf/wrappers.pb.h
  - basis_robot_input.pb.h

handlers:
  Update:
    sync:
      type: all
      # Run at 100fps
      rate: 0.01
    inputs:
      # All three inputs here are optional - we will run at 100hz regardless of the messages we get in
      /user_inputs:
        type: protobuf:basis::robot::input::InputState
        optional: True
        cached: True
      /servo/1/request_degrees:
        type: protobuf:google::protobuf::DoubleValue
        optional: True
      /servo/0/request_degrees:
        type: protobuf:google::protobuf::DoubleValue
        optional: True
    outputs:
      "/servo/0/current_degrees":
        type: protobuf:google::protobuf::DoubleValue
      "/servo/1/current_degrees":
        type: protobuf:google::protobuf::DoubleValue
{% endraw %}{% endhighlight %}

Finally - a little bit of complexity. This unit runs at 100hz, and picks up any inputs that were published since the last tick. We cache `/user_inputs` - it can run at a lower rate, and we're okay with reusing the last joystick input for 5 ticks - nobody will notice the difference.

### Doing things the hard way

The initial version of the servo looked something like this:
<pre class="mermaid">
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
  subgraph unit_/freenove/rpi_freenove_servo_driver["/freenove/rpi_freenove_servo_driver"]
    handler_/freenove/rpi_freenove_servo_driver::OnInputs[["OnInputs()"]]
    handler_/freenove/rpi_freenove_servo_driver::RequestState0[["RequestState0()"]]
    handler_/freenove/rpi_freenove_servo_driver::RequestState1[["RequestState1()"]]
    handler_/freenove/rpi_freenove_servo_driver::Update[["Update()
100Hz"]]
  end
handler_/freenove/rpi_freenove_servo_driver::RequestState1::/servo/1/request_degrees:::hidden x--/servo/1/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::RequestState1
handler_/freenove/rpi_freenove_servo_driver::RequestState0::/servo/0/request_degrees:::hidden x--/servo/0/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::RequestState0
handler_/freenove/rpi_freenove_servo_driver::OnInputs::/user_inputs:::hidden x--/user_inputs--> handler_/freenove/rpi_freenove_servo_driver::OnInputs
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/1/current_degrees--x /servo/1/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/0/current_degrees--x /servo/0/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
</pre>

This required storing each input to the unit and is less performant than letting the framework handle it.

### Optional and Cached

With `optional` and `cached`, the code is straightforward. `optional` lets a handler run without the tagged input. `cached` keeps an input around for future executions of the handler. You can use it to work around differences in publish rates while still having a single handler. In this example we use it for input, but another use might be for something like loading a map and publishing it at a low rate. The less often messages need published, the better.

<pre class="mermaid">
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
  subgraph unit_/freenove/rpi_freenove_servo_driver["/freenove/rpi_freenove_servo_driver"]
    handler_/freenove/rpi_freenove_servo_driver::Update[["Update()
100Hz"]]
  end
handler_/freenove/rpi_freenove_servo_driver::Update::/user_inputs:::hidden x--/user_inputs--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/rpi_freenove_servo_driver::Update::/servo/1/request_degrees:::hidden x--/servo/1/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/rpi_freenove_servo_driver::Update::/servo/0/request_degrees:::hidden x--/servo/0/request_degrees--> handler_/freenove/rpi_freenove_servo_driver::Update
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/1/current_degrees--x /servo/1/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
handler_/freenove/rpi_freenove_servo_driver::Update --/servo/0/current_degrees--x /servo/0/current_degrees::handler_/freenove/rpi_freenove_servo_driver::Update:::hidden
</pre>

I'll show the code here...

{% highlight cpp %}{% raw %}
class rpi_freenove_servo_driver : public unit::rpi_freenove_servo_driver::Base {
public:
  rpi_freenove_servo_driver(const Args& args, const std::optional<std::string_view>& name_override = {});

  virtual unit::rpi_freenove_servo_driver::Update::Output
  Update(const unit::rpi_freenove_servo_driver::Update::Input &input) override;

  PiPCA9685::PCA9685 pca;

  static inline constexpr size_t NUM_SERVOS = 2;
  std::array<double, NUM_SERVOS> current_state = {};
  std::array<double, NUM_SERVOS> requested_state = {};
};
{% endraw %}{% endhighlight %}
The declaration for our Unit is nothing special. We store the PCA9685 interface, and store both the current state and requested state for each servo.
{% highlight cpp %}{% raw %}
Update::Output rpi_freenove_servo_driver::Update(const Update::Input& input) {
  const float t = basis::core::MonotonicTime::Now().ToSeconds();

  if(input.servo_0_request_degrees) {
    requested_state[0] = input.servo_0_request_degrees->value();
  }
  if(input.servo_1_request_degrees) {
    requested_state[1] = input.servo_1_request_degrees->value();
  }

  if(input.user_inputs && !input.user_inputs->joysticks().empty()) {
    // If we have a joystick connected, use it
    constexpr float MAX_JOYSTICK_DEGREES_SEC = 180.0f;
    // Get the update rate for this handler
    const auto duration = handlers["Update"]->rate_duration; 

    // TODO: move to config
    constexpr size_t AXIS_IDXES[2] = {2, 5};

    const auto& joystick = input.user_inputs->joysticks()[0];
    for(int i = 0; i < NUM_SERVOS; i++) {
      const float delta = joystick.axes()[AXIS_IDXES[i]] * MAX_JOYSTICK_DEGREES_SEC * duration->ToSeconds();
      requested_state[i] = std::clamp(requested_state[i] - delta, -70.0, 70.0);
    }
  }
  else {
    // Otherwise, rotate back and forth
    // this logic will eventually get moved out to a separate unit
    requested_state[0] = (sin(t * 2.0)) * 70.0;
    requested_state[1] = (sin(t * 3.1)) * 60.0;
  }

  std::array<std::shared_ptr<google::protobuf::DoubleValue>, NUM_SERVOS> outputs;
  for(int i = 0; i < NUM_SERVOS; i++) {
    // TODO: we will eventually implement smoothing here
    current_state[i] = requested_state[i];
    outputs[i] = std::make_shared<google::protobuf::DoubleValue>();
    outputs[i]->set_value(current_state[i]);
    float ms = DegressToPWMMS(current_state[i]);
  
    pca.set_pwm_ms(8 + i, ms);
  }

  // Magic - convert from our array output to our output type
  return std::apply([](auto&&... args) { return Update::Output{args...}; }, std::tuple_cat(outputs));
}
{% endraw %}{% endhighlight %}
Notice - we don't have to store any messages, we don't have to write any synchronizer code. Very straightforward.

## rpi_freenove_mecanum_driver
{% highlight yaml %}{% raw %}
args:
  i2c_device:
    type: string
    default: "/dev/i2c-1"
  address:
    type: int32_t
    default: 0x40
threading_model:
  single
cpp_includes: 
  - google/protobuf/wrappers.pb.h
  - basis_robot_input.pb.h
  - basis_robot_state.pb.h

handlers:
  Update:
    sync:
      type: all
      # Run at 100fps
      rate: 0.01
    
    inputs:
      /user_inputs:
        type: protobuf:basis::robot::input::InputState
        optional: True
        cached: True

    outputs:
      /motor_state:
        type: protobuf:basis::robot::state::MotorState
{% endraw %}{% endhighlight %}

Same story as `rpi_freenove_servo_driver`. The only code of note is this:
{% highlight cpp %}{% raw %}
std::array<float, 4> XYTtoWheels(float x, float y, float theta) {
  constexpr float MAX_SPEED = 10.0f; // TODO: units
  // simple enough?
  // https://robotics.stackexchange.com/questions/20088/how-to-drive-mecanum-wheels-robot-code-or-algorithm
  // This isn't quite what we want, I think the /3.0 is bad
  return {
    -MAX_SPEED * (y+x-theta),
    MAX_SPEED * (y-x-theta), // Note: the hardware for this device has one reversed motor
    -MAX_SPEED * (y+x+theta),
    -MAX_SPEED * (y-x+theta),
  };
}
{% endraw %}{% endhighlight %}
Mecanum wheel control is really simple. This function takes in the x/y joystick input and the sum of the triggers (theta), and outputs the power to each motor to satisfy those inputs.

# Final thoughts

This was pretty simple to do - helped of course by the availability of other libraries out there. I'm looking forward to getting LiDAR working - and then either SLAM (with an IMU?) or a simple planning+controls stack. 