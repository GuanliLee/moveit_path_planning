# Isolate manual and inference operation code without changing the ROS protocol

Manual and inference collection keep separate operation entry points, controller
processes, runtime state, and logs. They deliberately retain the existing ROS
topics, service names, message types, and state-machine behavior; these are a
compatibility contract, not configuration owned by the new Web layer.

Because both flows address the same unchanged recorder service and the same
physical robot, the unified Web entry point allows only one collection flow to
be active at a time. Isolation therefore means that inference cannot source,
call, stop, or mutate the manual controller (and vice versa), not that either
flow gets a renamed ROS namespace.
