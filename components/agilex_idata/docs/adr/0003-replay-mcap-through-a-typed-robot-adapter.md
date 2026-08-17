# Replay MCAP through a typed robot adapter

Real-robot MCAP replay reads the existing recorded command streams and maps them through the same bounded arm, base, lift, stop, and emergency-safety runtime used by aligned HDF5 and LeRobot replay. The platform deliberately does not use broad `ros2 bag play` because recorded state and JSON action topics are not the robot command interfaces and republishing the entire bag would create unsafe topic conflicts.
