# Robot-Summer
Creating an autonomous driving robot to race in a competition.

The competition has a theme of mario cart. The goal is to drive around a track (seen below), complete laps, and collect boxes.

![image](https://github.com/HudsonNock/Robot-Summer/assets/111708296/c34a69c2-2ec8-4735-ba39-e0346ec709a5)

Our strategy was to go very fast, avoiding the boxes, and taking the path seen in red below which includes jumping off the ramp. We think that if this would be done correctly the lack of boxes could be made up for with a faster lap time since the competition runs for two minutes.

![image](https://github.com/HudsonNock/Robot-Summer/assets/111708296/5e538c97-165c-48d1-9fa8-bd6d34ca74d2)

Because we were jumping off a ramp, the structural integrity of our robot would have to be very high, therefore, we built the robot out of metal. To steer the robot we implemented ackerman steering and had rear wheel drive using two motors. Because there were two motors, we could make even sharper turns by turning the back wheels at different speeds.

![IMG_3906](https://github.com/HudsonNock/Robot-Summer/assets/111708296/ad2d5aca-19f7-4d4b-8a0c-8aa2fedae961)

Instead of line following, we choose to navigate using an IMU for orientation and counting the number of wheel rotations. Each time we counted a 1/5th of a wheel rotation, we updated our new position as if we moved along the edge of a circle where our previous and new orientation were tangent to the circle we moved along, and the arc length between the initial and final position was determined empirically. We then mapped the path we want the robot to take in software, then used PID to ensure it followed the track based on its estimated position.

https://github.com/HudsonNock/Robot-Summer/assets/111708296/dd74e8d9-3797-4fea-90d0-76ce23e65052

As seen above, one problem encountered was after the jump we couldn't rely on our data anymore because there was too much uncertainty in our position and orientation. To solve this, we implemented a magnetometer to recalibrate the IMU, as well as distance sensors and sonar sensors. The sonar sensor was used to detect the bridge so as to recalibrate the position of the robot along one line (entering and exiting the bridge). The distance sensor was used to determine its distance to the ramp wall, which would calibrate along a perpendicular line to the bridge. We also attempted to calibrate our position by detecting when we were on and off rocks.

However, we had major inaccuracies in the distance sensor and the magnetometer which made it fail recalibrations. Unfortunately we could not resolve this issue before the competition deadline. Our hypothesis was that the noise from the motors and the circuits generated large magnetic fields comparable to that of earth's magnetic field which interfered with our orientation. We tried to resolve this by placing the magnetometer far from the circuity, however, we think it was still too close as the robot was relatively small. We independently tested the distance sensor, and the resulting signal was very noisy when the laser was shaken. We think this was the main problem with the distance sensor and we were not sure how to fix this problem in time.




