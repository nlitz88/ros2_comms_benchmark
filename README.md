# ros2_comms_benchmark
Package for stress testing and evaluating ROS2 node communication.

This package is comprised of nodes, launch files, and analysis scripts help you
sniff out weak links in your ROS 2 system. These are meant to help you answer
common questions like:

- What percentage of messages being published are actually being received by a
  subscriber using composition vs. without composition?
- What frequency can I configured my camera node to publish images at before my
  Jetson starts choking?
- Is the rosbag2 recorder dropping images or does realsense ros just suck?
- Did I do more harm than good by messing with my publisher's QOS settings?
- Should I switch DDS implementations because I think that will be the silver
  bullet that solves all my problems?

And more!

Currently, these benchmarking nodes just pass image messages around to mimic a
realistic, intense workload. It would be great to add nodes that publish mock
point clouds, too (or any message type that is typically huge). Likewise, maybe
mock imu messages for passing around high frequency sensor measurements. While
these aren't perfectly representative of real use cases, they're darn close, and
should help you smoke test out any major issues.

## Nodes
### benchmark_publisher

### benchmark_subscriber

## Launch Files

### Composed single threaded executor
### Composed multie threaded executor
### Not Composed
### Publisher only (for remote host)
### Subscriber only (for remote host)
### Launch files also examining rosbag2 performance?

Plan: Create a suite of configuration files that can applied for each
configuration (I.e., whether nodes are composed or not) that control test
parameters such as published image size, frequency, publisher QOS settings, etc.
Something like that.

## Analysis Scripts

TODO: Flesh out what analysis scripts I want to see. Right now, I'm thinking I
should look at the very least:
- Percentage of messages dropped.
- Mean and stdev message latency
- Mean and stdev delay between message headers (I.e., time each message was
  born). Also, prob want min, max, and median for this metric as well.
- Message latency vs time plots.
- Header difference vs time plots.

While I don't want to make anything too bespoke, I do want to eliminate as many
variables as possible here, so I'm thinking the above data points will be
generated by the subscriber writing results to a file on disk, or by publishing
a diagnostics message.

Further, if we want more detailed timing diagnostics, I think that's where I'll
lean on ros2 trace tools, and do analysis on the traces produced by that. I.e.,
getting metrics like:
- The mean and stdev for the elapsed time between the publisher's timer
  callback invocations.
- elapsed time vs time plots.

It would be great to simultaneously have plots of various IO devices vs time.
I.e., like:
- Utilization of each network interface w.r.t. time. Maybe also network
  utilization of each PID vs. time.
- Utilization of each disk vs. time. Also per PID disk usage vs. time.
- CPU Utilization of each PID vs. time
- Overall memory utilization over time
- GPU Utilization of each PID vs. time

and so forth. While these should really be diagnostics available on any system
anyway, I think they would be particularly useful for post-mortem analysis
systems like this.

Also, should add a callback in the subscriber node for detecting time jumps. If
the subscriber node ends up using the system clock, which is not gauranteed to
increase monotonically (because of sporadic time synchronizations via NTP or
some other mechanism), then it might think there was a big gap in messages, when
in reality, the system time reference it is using has jumped.

Having said that, I think another part of this analysis should be plotting the
system time vs time time steps, starting a t=0 based on a monotonic, steady
count. The system clock should be
recorded to the /clock topic of a ros bag anyway, so this should be reasonable
to plot. In addition to the time jump callback, there should be a post
processing script that creates a time jump vs time plot, or something like that.