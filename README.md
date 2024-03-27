# oriens

oriens is (an other) ROS2 package which provides IMU and MARG fitlers for orientation estimation. Created for fun and learning, it provides some ready-to-use common filters.

### Filters
Current implmented filters are:
- Madgwick filter (IMU, MARG)
- Complementary filter (IMU, MARG)

For IMU versions, a _no-motion-no-integration_ simple method is provided to avoid gyroscope drift when estimating yaw angle.

### Docker
Scripts to build an image and run a container image are provided in the dedicated folder.

### Name
The word "oriens" comes from Latin and "indicates the east, the cardinal point where the sun rises".
