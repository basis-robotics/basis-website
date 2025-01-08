---
layout: post
title: Basis is shutting down
author: Kyle Franz
---

Basis as a company is shutting down. Thomas and I gave it a good shot but - we have no funding, and after our release, not one person tried our software.

# Good software, bad company

## Building backwards

What happened? Basically, what we have are the pretty good bones of a distributed framework, but there's no way we can see this working economically. I built everything backwards - I had an idea that I wanted to build, built it, and then tried to get others interested. That's fine for OSS (I'm happy I built it, and I'll use it), but not great for sustainting a business. 

One of the VCs we talked to specifically asked where our plan for building a community was. At the time I didn't get it (why would I waste runway on that?! who would join a community for a thing they couldn't try?), but now I see that he was looking for some form of validation - I'd assumed that releasing the framework would get people to try it, rather than paving the way for success first. Conventional startup wisdom seems to be to find PMF first, then build. We did the reverse. (Matt! You were right, we just didn't get it at the time.)

## Focusing on the wrong thing

Late in the process, we realized that we were somewhat barking up the wrong tree. We'd focused very much on deterministic replay and testability as the framework's key features. In our experience looking at larger robotics companies, these are important to have. However - our initial customers wouldn't care about those things. They care about how quickly they can get off the ground and get features out the door. 

There's a reason people use ROS - it's really great for getting your robot prototyped. We could have focused our "marketing" on doing that, but even better - this was an area we are better than ROS in, in several ways (not having to use a custom build system, easily supporting new platforms, abstracting away system software concerns from your roboticists, etc). This was another bit of feedback we got from a VC - they recognized that ROS is a problem, but didn't see how believe it was slowing down their robotics companies. We should have swapped our marketing much sooner, especially as we got several comments on release that "something easier get started with than ROS" was a great idea.

## Difficult to capture initial customers

There's another problem here - who _are_ our initial customers? We'd had two ideal companies in mind: fresh companies with not much code, and companies stuck on ROS1 but not yet moved to ROS2. In retrospect, these are difficult customers to find.

The first group isn't super loud about existing, and moreover, they are busy and unlikely to want to bet their company on new tech, unless they really know what they want (in our entire time of searching, we found one of these companies, who is/was still a promising lead as of this post, but see the next section for how this doesn't help...). 

The latter group is just too damn hard to capture. Ignoring the fact they have a lower tolerance for missing features, they more than likely have started to swap to ROS2 anyhow. They won't want to roll back that move, and aren't likely to want to immediately move a second time after making the jump. We talked with a few companies still on ROS1 and "happily" there - if fit their needs, they were fine with eating the maintinence cost of a fork, and didn't need anything we could provide. Beyond all this, it's a huge ask to get companies to just try our stuff out. Robots are pretty specialized, and we don't have a deep enough back cataloge of modules to be able to just toss some drivers and a planning algorithm together and get a quick prototype working for a new user wanting to try things. We probably could have done better here, and made more compelling demos.

## Economics

Lastly - this whole thing never actually made sense from an economic perspective. The whole idea was that companies should be willing to pay for a better middleware. So far...I haven't seen any evidence of it, though. People are willing to pay for a lot of things that have free alternatives, and willing to pay for robotics software, too. Foxglove appears to be succeeding in the visualization/data management space. For some reason, people are willing to throw millions a year at Applied Intuition for simulation. 

Why can't middleware be the same? We theorized that using Basis could save at least a system software hire, that has to be worth something, right? Turns out, not so much. I have some theories around this, but I'm guessing it's just too scary to move to a new framework, even for free. We had one friend in the industry suggest that companies might pay $1MM/year for a performance focused framework (I wish it were true, but nobody has come up to me with promises saying they would). Our best plan was to get a little bit of money short term in (discounted) contract integration work, and then make it up years down the line with some sort of game-engine-like fee structure. This still has a few big holes:

1. How do we structure the contract/license so that customers are protected from Basis Robotics having bad behavior down the line? Putting on my customer hat, I'd be very afraid of basing my business on someone else's platform - they can screw me later and there's nothing I can do! This happens a fair amount in the mobile space w/Apple and Google. Those companies deal with it, but smaller developers often feel the pain.
2. How is pricing scaled? If it's based on revenue, we lose out on all money from the next Cruise or other company that _has_ a ton of money, but makes zero revenue. If it's based on robot count or developer count, it's very hard to make a one size fits all contract. This sucks from the point of view of having clear pricing.

Even if we find a price structure and contract that fits, the math on profit is grim. And given we don't have evidence people will pay in the short term, that rules out bootstrapping.

## What's happening to the framework?

Basis as a piece of software will continue to exist, as a hobby/Open Source project. I'll make the license plain Apache 2.0, and open the `determinsitic_replay` repo that makes deterministic playback possible (and eventually pull it forward to be compatible with `main`, integrate it within the `basis` repo, etc, etc). I'll still work on my little test robot, for fun. The next thing to do is to integrate the LIDAR I bought and put some SLAM algo on top - there's a few out there, but once the [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release) releases Part 2, I might try to do my own.

# Why didn't we...

## Get Angel funding?

Angel funding won't solve the user problem. Again, not a single person tried our software after we released it. Maybe we could have done so and paid a marketing genius to help us out. Beyond that, if we're going to get funding I want enough that I can draw a small salary to pay rent. Before that it's not really worth it, if we had enough traction I'd much rather have borrowed/gotten investment from friends/family. The numbers are low enough for angel funding that I can make it back in a short time at a FAANG or AI company to make them whole, if we borrowed and busted.

## Pivot?

We could certainly pivot to making a robot, instead. I have a few ideas (putting it out there - some sort of Foundational Model based handyman robot with interchangable hands/fingers will come some day, I think). I think there's also space for improvement for robotics deployment and telemetry (though the space isn't as empty as middleware was). But I'm really loathe to spend another three to six months grinding and fundraising (again) - my savings are drained (they were low already due to some poor choices with stock options from Embark Trucks). We aren't going to get kicked out of our apartment or anything - my wife has savings and income, but I need a good reason to start dipping into her savings as well.

## Build on top of ROS?

I don't want to. I don't like their development model: everything is way too distributed, designed by committee, it's impossible to know who really owns what or what is good quality. [My PR to tf2 removing extra Matrix->Quaternion conversions still hasn't merged...](https://github.com/ros2/geometry2/pull/741). Beyond that, I don't think the user story really makes sense. The static analysis and replay benefits that Basis has won't work for things like MoveIt or Nav2 - they are too large, I'd never convince them to use my bindings on top, and the API surface is too large to cover as a fork. It might have been a solid strategy to start with (one of the YC partners asked why we weren't), but at this point - too late. Maybe someday I'll go back and add a ROS backend to the code generator.

## Integrate AI?

This would have probably been a solid play to get VC money, if I were more dishonest. I did wrack my brain for how AI could help, and at best I could come up with is "declaring all of each unit's inputs and outputs makes it easier for AIs to understand the codebase", which is true for humans as well. I think there is probably a need for a good production framework for working with Foundational Models, but I'd need to invest in more hardware and pay a platform to train models on (as well as skilling up on modern AI, which I suppose I'll have to do eventually, anyhow).

# Conclusion

I learned a hell of a lot out of this. It was even more stressful than I expected, and we never even had money change hands. I went from being super nervous at every pitch to being pretty relaxed and happy to talk about what we were building. Got a whole lot better at networking at conventions, gave my first talk in front of other companies at an event. I got to set up the bones of a company from scratch. I got to get the idea that's been knocking around my head for the past three or four years out of it and into code. I feel lucky that I had the opportunity for all of this and I'm not unhappy for trying.

Given the opportunity, I'd do it again.

## What's next?

It sounds like Thomas is taking some time off - he deserves it. It's been a fun time, and I'm glad to have had him at my side.

I'm now open to contract and full time work. You can see the entire corpus of what I've worked on over the past 7 months on our github (please keep in mind that normally I'd do a bit better with tests/docs!). I'd love to stay in robotics but open to other areas as well. Please reach out at kyle@basisrobotics.tech