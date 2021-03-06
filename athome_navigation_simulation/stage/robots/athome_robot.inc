define laser ranger
(
  sensor
  (
    range [0.05 30.0]
    fov 180
    samples 720
    resolution 1
    # size [ 0.06 0.15 0.03 ]
    size [ 0.06 0.06 0.05 ]
  )
  # generic model properties
  color "black"
)

define diff_drive_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.07 0.07 0.0 3.5]
  size [ 0.588 0.455 0.16 ]
  # origin [ 0.0 0.0 0.0 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "diff"
  localization "odom"

  # spawn sensors
  # laser(pose [ -0.1 0.0 -0.11 0.0 ])
  laser(pose [ 0.27027 0.0 0.204 0.0 ])
)
