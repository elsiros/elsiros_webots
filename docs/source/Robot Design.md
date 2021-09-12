# Robot Design
Standard robot proto used for simulation is Robokit1.proto. It is designed as virtual copy of real robot.
In order to represent real robot all technical features of virtual model were copied from technical description of real robot including servo torque, servo speed, servo mass, location of camera, camera resolution, aperture of camera lens, number and location of accelerometers, weight distribution throughout of body.

![Robokit_1 (2)](https://user-images.githubusercontent.com/57300002/133002574-91c5ca77-854d-4866-92b9-37d8ef317cd5.png)

Virtual model has 2 IMUs named "imu_head" and "imu_body".
Real robot is capable to recognize coordinate of itself at soccer field with accuracy ±15cm, it is capable to recognize obstacles and ball. 
Recognition of distance and course to ball and obstacles is made through machine vision algorithms. Robot detectds distance and course to object with accuracy mainly dependent on camera accuracy.
Good calibrated robot detects course to object with accuracy ± 0.01 Radian. Accuracy of distance recognition non-linearly depends on distance to object, with best accuracy at minimum distances which is ± 5mm.
Distance measurement accuracy at distance 1m is ± 25mm, at distance 2m is ± 100mm
High accuracy of distance and course measurement are provided due to equipment and smart algotihms used for direct measurements.
Localization on soccer field is provided by measurements of cource and distence to goal posts, field marking, green field border, odometry, measurement of IMU.
All localization data is accumulated with historical data and processed by patricle filter algorithm in order to achieve better accuracy than measurement of individual object.
Simulation procedure in Webots is orgaised in way when simulation of physics and motion is provided together with camera image scanning. 
Procedure of image scanning and transfer to outside from Webots greatly reduces simulation speed. 
In order to keep simulation speed at acceptable rate it was decided to skip procedure of scanning images by camera in simulation and replace it with direct request- report procedure.
Following data are reported to Robokit1 robot from simulation by request: distance and course from robot to ball, distance and course from robot to other players, self coordinate and orientation on field.
Requested data before reporting to robot are mixed with random values in order to provide same accuracy which usually real robot detects from camera image. 
Above procedure is called "blurrer". Above procedure provide increasing is simulation speed up to 10 times. This know-how of ELSIROS provides games to be played at laptop computer with nearly realtime speed.

Below is technical description of real Robokit robot.

![Robokit description-0](https://user-images.githubusercontent.com/57300002/133003954-6c4c3827-8f13-4a8a-9528-364fb16dd564.jpg)

![Robokit description-1](https://user-images.githubusercontent.com/57300002/133003963-c74acbd5-3d44-4ffd-adee-4b687d35d40c.jpg)



