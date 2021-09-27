# TUTORIAL 1
Let’s try to change strategy of “forward” role using matrix stored strategy_data.json.
If you launch first game which was supplied in installation package you may notice that red player 2 
is granted first kick off. According to rule player which is not granted kick off is not allowed to enter 
central circle within 10 seconds after game start therefore blue player 2 stands outside of central 
circle.  Red player 2 kicks ball directly to blue player 2. Direction of kick is not beneficiary in this case. 
It makes sense to change kick direction. There are several ways to change kick direction. Here we 
consider how to change it using strategy matrix. 
1)	Open DOS console window by command “cmd”;
2)	Type in command prompt “cd C:\Elsiros\controllers\SAMPLE_TEAM”. This will change current directory to “C:\Elsiros\controllers\SAMPLE_TEAM>”
3)	 Type in command prompt “python strategy_designer.py” 
4)	When strategy_designer.py will be launched and you can see following window:

![image](https://user-images.githubusercontent.com/57300002/134931725-e3531b97-f34f-4dd7-932d-4d429c653ff3.png)

Soccer field is divided to sections. In each section located short line segment with small circle 
representing direction and power of kick. 
5)	Click on “Load File” button and load following file: “C:\Elsiros\controllers\SAMPLE_TEAM\Init_params\ strategy_data.json”.
Following window will appear:

![image](https://user-images.githubusercontent.com/57300002/134931964-3037e817-4d31-4e54-b70d-d6227b852018.png)

6)	Click on field section which is 7-th from top and 9-th from left.

![image](https://user-images.githubusercontent.com/57300002/134932056-f8774921-5648-425a-904b-685c534d308e.png)

7)	Yellow line represents supposed distance of kick.
8)	Drag “YAW” slider to right or type in input window 45 in order to select new kick direction to be 45 degrees
 of Centigrade. Drag “POWER” slider to right or type 3 in input window in order to select new attenuation of kick power.
 
![image](https://user-images.githubusercontent.com/57300002/134932186-3538252b-b48e-4234-90ea-e66fe6b79c54.png)

9)	After selecting new values new data of section have to be recorded to matrix prior to undertaking similar modifications to other section. For this click on “Record” button.
10)	Click on section which is next to current and located 7-th from top and 10-th from left.

![image](https://user-images.githubusercontent.com/57300002/134932274-2b53660f-3469-48e4-aaae-b87b64969a79.png)

11)	Click on “Record” button one more time in order to use same direction and power for this section. 
12)	In order store changes into strategy_data.json file click on button “SaveExit”
13)	Launch game and observe result of changes made into strategy. 
