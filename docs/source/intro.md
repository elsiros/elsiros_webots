# ELSIROS Intro

Soccer game of Humanoid robots is becoming popular between teams in education and academic society. This type of competitions generates many scientific research points. For students in high schools, colleges or universities in general there are 3 obstacles preventing them to set up and maintain a humanoid robot soccer team:

1) extremely high price of humanoid robots,

2) too complex algorithmic problems to be solved at initial steps of setting robot into game,

3) small number of teams in one geographical location – means every game is possible only together with travelling to far distance with relatively high travel charges.

All 3 obstacles can be eliminated if teams start their Humanoid Soccer experience from ELSIROS. Following advantages are obtained with using ELSIROS:

1) ELSIROS is free of charge and open source.

2) Most difficult parts of humanoid robot development which are beyond of school or college program like Inverse Kinematics, walking engine, path planning, detecting of ball and obstacles and localization are provided readymade in source codes. Example of playing strategy which proved to be a leading strategy at games of real robots is provided for study and improvement.

3) In order to participate in competitions or challenges it is not necessary to travel. Participating teams can compile their source code into executable binary code which is safe against source code leaking and upload to referees server.

4) Teams don’t suffer from backlashes, mis-tunings, mis-calibrations because models are tuned, calibrated and free of backlashes.

5) Strategy modules developed by teams will be ready to be used on real robots which teams may decide to build or to buy from market in future.

6) It is not necessarily powerful servers for running training games, simulation can run even at laptop.

ELSIROS is created by Humanoid Robot Soccer team “Starkit” from MIPT (Moscow) after winning Robocup World Championship 2021.
ELSIROS is free of charge and open-source platform pretending from one side to help new research groups to enter into Humanoid Soccer Competitions world, from other side to host virtual games.
ELSIROS comprises of following components:

1) Webots simulator (to be downloaded from vendors’ site);

2) Primary robot model, which is a virtual model of existing physical robot – a winner of international humanoid soccer challenges in 2019, 2020 and 2021 used by team “Robokit”;

3) Soccer simulation environment for simulator;

4) Autonomous/Human referee program and game controller;

5) Robot controller software pack capable to play games;

Teams can participate in competitions with Robokit robot and use it for study of basics of programming of strategy of soccer game for humanoid robots. This is convenient instrument for study of Artificial Intelligence in schools, colleges and universities.
Programming of robots is supported with Python 3 language. But in case of necessity also C, C++, Java or MATLAB languages can be used for your convenience

Structure of robot controlling software is built for 4 level of robot developers: Beginner, Medium, Advanced and Expert level.
Beginner level developers can access to programming of strategy.py file with purpose to change current robot behavior in game play. Initially supplied source code represents strategy of game used by leading Russian team at National championship 2021. Video of this game can be found under link below:

    https://youtu.be/AmfKpkL2MUc

Medium level developers can try to improve launcher.py module. This module stands for detecting game state, player state and team state, managing players role and their starting positions.
Advanced developers can try to modify other modules of source code which are responsible for inverse kinematics, motion, localization, robots’ path planning.

Expert level developers are allowed to compose their own model of robot and use their own controller software with or without using source code included into ELSIROS open-source package.
In order to be admitted to competition program team providing virtual model of their own robot design have to be qualified. Main requirement to robot model is that virtual model and real robot must have the same technical performance in all details. Special requirements to robots’ PROTO which appear from simulation environment can be sent after special request.

If you are a mentor of potential Humanoid Robot Soccer team you can easily start-up with your team. It is necessary simply download and install ELSIROS in your computer. There is executable version for Windows 10 or source code for Linux. Please follow instructions and sample code will play soccer just after installation. Please use your favorite Python 3 IDE for improving players’ source code and you are ready to Virtual Humanoid Robot Soccer challenges in simulation. You can train your games at your laptop computer.
