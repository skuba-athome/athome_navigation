define laser ranger
(
  sensor
  (
    range_max 30.0
    fov 180.0
    samples 720
  )
  # generic model properties
  color "black"
  # size [ 0.06 0.15 0.03 ]
  size [ 0.06 0.06 0.05 ]
)

define diff_drive_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]
  size [ 0.588 0.455 0.16 ]
  # origin [ 0.0 0.0 0.0 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "diff"

  # spawn sensors
  # laser(pose [ -0.1 0.0 -0.11 0.0 ])
  laser(pose [ 0.27027 0.0 0.204 0.0 ])
)
