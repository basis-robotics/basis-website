---
layout: page
---

Basis is a production-focused robotics development framework by Basis Robotics, built on a pub-sub architecture designed for deterministic testing. Unlike ROS and similar frameworks, Basis abstracts publishers and subscribers from the developer, using a model of `Inputs + Conditions (synchronizer) -> Handler (your code) -> Outputs`. You declare the messages and conditions that your code responds to, allowing the framework to automatically manage all underlying publishers, subscribers, and message routing as inputs and conditions trigger your code to process data and produce outputs.

<a href="https://github.com/basis-robotics/basis" style="display: inline-block; padding: 5px 22px; font-size: 20px; background: #eee; text-decoration: none; color: #444; border-radius: 5px;">
Get Basis <img src="assets/images/github-mark.svg" style="vertical-align: center; width: 30px; height: 30px; padding-top:11px;"/>
</a>

### Why Choose Basis?

#### Rapid Prototyping

Easily experiment and iterate with our intuitive plug-and-play modules, speeding up your development process from concept to prototype.

#### Reliable Production Code

Ensure your applications meet the highest standards with features like deterministic replay and advanced synchronization for robust performance.

#### Resource Efficiency

With a low CPU footprint and efficient message transport, Basis is perfect for both resource-constrained environments and high-demand applications.

#### Versatile Language Support

Develop in your preferred languages with current bindings for Python and Rust, and more languages coming soon.

#### Plug-and-Play Serialization and Message Transport

Customize configurations to suit your project needs with support for ROS, Protocol Buffers, and FlatBuffers. More options are on the way.

#### Minimal Dependencies

Simplify OS upgrades and package updates with minimal external dependencies, resulting in smaller symbol files and reduced link time.
