# CamelBot
Camelbot is thus named due to its longevity - with a 40Ah battery and around 3A of current drain when traveling over level terrain, this device can last over 12 hours on a single charge. Also, it kind of looks like a camel with two humps.

So far, the items implemented are:
- manual control over bluetooth
- automated movement based on heading and ultrasonic rangefinders (still a WIP)
- path tracking, cubic spline paths
- sensor feedback and csv storage (IMU, GPS, rangefinders, battery voltage)
- local reference frame transformations, positioning
- python scripts for the above, android app for control + commands

TODO:
- better automated path tracking
- waypoint finder w/ google maps interface
- device locater w/ google maps interface
- user following mode
- occupation map obstacle avoidance (update to current behavioral model)
- calibration for magnetometer - may be subject to motor interference, so this is still iffy. GPS can determine direction also
- upload the app (app is working, but still in pre-alpha)

Any Questions, Comments, or Concerns, please email me at :

danielwest7@gmail.com


