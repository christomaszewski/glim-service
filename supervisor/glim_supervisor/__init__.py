"""In-container supervisor for one GLIM instance.

It owns the GLIM process and the operational state (active = a mapping session is running;
standby = no session, map saved), composes each session's GLIM config from the pinned upstream
defaults plus the launcher-rendered instance file, and reports fleet health on /diagnostics.
Commands arrive over a local Unix socket (the launcher's standby/activate/state verbs and the
container healthcheck) or through the supervisor's ROS lifecycle node (mission software).

Only the ROS integration (ros_iface) imports rclpy; everything else is plain Python, so the
state machine, config composition and health logic are testable without ROS.
"""
