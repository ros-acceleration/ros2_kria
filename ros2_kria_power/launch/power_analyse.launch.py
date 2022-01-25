#       ____  ____
#      /   /\/   /
#     /___/  \  /   Copyright (c) 2021, Xilinx®.
#     \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#      \   \
#      /   /
#     /___/   /\
#     \   \  /  \
#      \___\/\___\
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import bt2
import sys
import datetime
import os


debug = True  # debug flag, set to True if desired


def get_change(first, second):
    """
    Get change in percentage between two values
    """
    if first == second:
        return 0
    try:
        return (abs(first - second) / second) * 100.0
    except ZeroDivisionError:
        return float("inf")


# Create a trace collection message iterator from the first command-line
# argument.
msg_it = bt2.TraceCollectionMessageIterator(
    str(os.environ["HOME"]) + "/.ros/tracing/power_capture"
)

# Iterate the trace messages and pick those ones of interest
power_msgs = []
for msg in msg_it:
    # `bt2._EventMessageConst` is the Python type of an event message.
    if type(msg) is bt2._EventMessageConst:

        # An event message holds a trace event.
        event = msg.event

        # Only check `sched_switch` events.
        if event.name == "ros2_acceleration:kria_power_dt":
            power_msgs.append(msg)

# Last event's time (ns from origin).
last_event_ns_from_origin = None

# lists to store power measurements and time differences considering two
# scenarios:
#   - using_msg: using the time difference obtained from the trace (calculated in the process)
#   - using_trace: using the time difference calculated from the trace (actual real time)
using_msg_power = []  # watts
using_msg_dt = []  # seconds
using_trace_power = []  # watts
using_trace_dt = []  # seconds

for msg in power_msgs:
    # Get event message's default clock snapshot's ns from origin value.
    ns_from_origin = msg.default_clock_snapshot.ns_from_origin

    # Compute the time difference since the last event message.
    dt = 0

    if last_event_ns_from_origin is not None:
        dt = (ns_from_origin - last_event_ns_from_origin) / 1e9

        using_msg_power.append(msg.event["power"] * msg.event["dt"])
        using_msg_dt.append(msg.event["dt"])
        using_trace_power.append(msg.event["power"] * dt)
        using_trace_dt.append(dt)

        if debug:
            #
            # debug
            #
            # Create a `datetime.datetime` objects from
            # `ns_from_origin|last_event_ns_from_origin` for
            # presentation. Note that such an object is less accurate than
            # `ns_from_origin` as it holds microseconds, not nanoseconds.
            date_past = datetime.datetime.fromtimestamp(last_event_ns_from_origin / 1e9)
            date_now = datetime.datetime.fromtimestamp(ns_from_origin / 1e9)
            #
            fmt = "{} → {} (+{:.6f} s): using-dt ({}), using-tracing ({})"
            print(
                fmt.format(
                    date_past,
                    date_now,
                    dt,
                    using_msg_power[-1],
                    using_trace_power[-1],
                )
            )

    # Update last event's time.
    last_event_ns_from_origin = ns_from_origin


fmt = "using_msg ({:.6f} s): {} W | using_trace ({:.6f} s): {} W"
print("TOTAL:")
print(
    fmt.format(
        sum(using_msg_dt),
        # sum([pwr * dt for pwr, dt in zip(using_msg_power, using_msg_dt)]),
        sum(using_msg_power),
        sum(using_trace_dt),
        # sum([pwr * dt for pwr, dt in zip(using_trace_power, using_trace_dt)]),
        sum(using_trace_power),
    )
)
print(
    "Change: "
    + str(
        get_change(
            # sum([pwr * dt for pwr, dt in zip(using_msg_power, using_msg_dt)]),
            # sum([pwr * dt for pwr, dt in zip(using_trace_power, using_trace_dt)]),
            sum(using_msg_power),
            sum(using_trace_power),
        )
    )
    + " %"
)


def generate_launch_description():
    return LaunchDescription()
