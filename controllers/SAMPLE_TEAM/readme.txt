Preapration of files for export to competition. 
step-by-step guide.

If you care about leaking of your source code to competition organizers you can compile your code by nuitka.
Compiled binary code doesn't contain source code. Compiled code works faster than python original code
 running through Python interpreter. In order to use nuitka compilator install nuitka to your system from link:
https://nuitka.net/pages/download.html
Before installing nuitka check version of Python and choose correct installer.
After successful Nuitka installation follow steps below:  

1) create copy of directory "SAMPLE_TEAM";
2) rename new directory by your team name, for example "MY_TEAM" of any other name;
3) launch OS console by command "cmd";
4) in console change directory by command: "cd C:\Elsiros\controllers\MY_TEAM";
5) type in command prompt:"python -m nuitka --follow-imports --standalone --onefile main_pb.py";
6) as result of this compilation there must be created "main_pb.exe" file;
7) edit "team1.json" or "team2.json" file in directory C:\Elsiros\controllers\referee\" 
	by replacing ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," with ""robotStartCmd": "MY_TEAM/main_pb.exe","
8) launch game and  make sure that controller is working correctly;
9) delete all *.py files inside MY_TEAM directory with keeping all other files and directories;
10) delete directories: "main_pb.onefile-build", "main_pb.dist", "main_pb.build"
11) delete all directories with name "__pycache__";
12) copy modified file "team1.json" or "team2.json" to "MY_TEAM" directory.
13) compress "MY_TEAM" directory into zip archive;
14) upload "MY_TEAM.zip" file to any cloud storage;
15) send reference to file "MY_TEAM.zip"in cloud storage to competition organizer.
16) composed archive must contain only "*.exe" or "*.json" files.

If you don't care about leaking of your source code then you have to fulfill following steps:

1) create copy of directory "SAMPLE_TEAM";
2) rename new directory by your team name, for example "MY_TEAM" of any other name;
3) edit "team1.json" or "team2.json" file in directory C:\Elsiros\controllers\referee\" 
	by replacing ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," with ""robotStartCmd": "MY_TEAM/main_pb.py","
4) launch game and  make sure that controller is working correctly;
5) copy modified file "team1.json" or "team2.json" to "MY_TEAM" directory.
6) compress "MY_TEAM" directory into zip archive;
7) upload "MY_TEAM.zip" file to any cloud storage;
8) send reference to file "MY_TEAM.zip"in cloud storage to competition organizer.

???????????????????? ???????????? ?????? ???????????????? ???? ????????????????????????.
?????????????????? ????????????????????.

???????? ?????? ?????????????????? ???????????? ???????????? ?????????????????? ???????? ?????????? ?????????????????????????? ????????????????????????, ???? ???? ???????????? ????????????????????????????
?????? ?????? ?? ???????????????? ?? ?????????????? nuitka. ???????????????????????????????? ?????? ???? ???????????????? ?????????????????? ????????. ???????????????????????????????? ????????????????
 ?????? ???????????????? ??????????????, ?????? ???????????????? ?????? ?????????? ?????????????????????????? Python. ?????? ?????????????????????????? ?????????????????????? nuitka
 ???????????????????? ?????? ???? ???????? ??????????????, ?????????????????????? ?????????????????? ????????????:
https://nuitka.net/pages/download.html
?????????? ???????????????????? ?????????????????? ???????????? ???????????? ???????????? ?? ???????????????? ???????????????????? ????????????????????????
?????????? ???????????????? ?????????????????? nuitka ?????????????????? ???????? ????????: 

1) ???????????????? ?????????? ???????????????????? "SAMPLE_TEAM";
2) ???????????????????????? ?????????? ???????????????????? ???????????? ?????????? ??????????????, ???????????????? "MY_TEAM" ?????? ?????????? ???????????? ????????????;
3) ?????????????????? ?????????????? ???????????????????????? ?????????????? ???????????????? "cmd";
4) ???????????????? ???????????????????? ?? ?????????????? ????????????????: "cd C:\Elsiros\controllers\MY_TEAM";
5) ???????????????? ??????????????:"python -m nuitka --follow-imports --standalone --onefile main_pb.py";
6) ?? ???????????????????? ???????????????????? ???????????? ?????????????????? ????????: "main_pb.exe";
7) ???????????????? "team1.json" ?????? "team2.json" ???????? ?? ???????????????????? C:\Elsiros\controllers\referee\" 
	?????????? ???????????? ???????????? ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," ?????????????? ""robotStartCmd": "MY_TEAM/main_pb.exe","
8) ?????????????????? ???????? ?? ??????????????????, ?????? ?????????????????? ???????????? ??????????????????;
9) ?????????????? ?????? ?????????? "*.py" ?? ???????????????????? "MY_TEAM" ???????????????? ?????? ?????????????????? ?????????? ?? ????????????????????;
10) ?????????????? ????????????????????: "main_pb.onefile-build", "main_pb.dist", "main_pb.build"
11) ?????????????? ?????? ???????????????????? ?? ???????????? "__pycache__";
12) ???????????????????? ???????????????????? ???????? "team1.json" or "team2.json" ?? ???????????????????? "MY_TEAM".
13) ?????????????????????????? ???????????????????? "MY_TEAM" ?? zip ??????????;
14) ?????????????????? "MY_TEAM.zip" ?? ???????????????????????????? ???????????????? ????????????;
15) ?????????????? ???????????? ???? ???????? "MY_TEAM.zip" ?? ???????????????? ?????????????? ???????????????????????? ????????????????????????.
16) ???????????????????????????? ?????????? ???????????? ?????????????????? ?????? "*.exe" ?????? "*.json" ??????????

???????? ?????? ???? ?????????????????? ???????????? ???????????? ?????????????????? ???????? ?????????? ?????????????????????????? ????????????????????????, ?????????? ???????????????????? ?????????? ????????????????
?????????????????? ????????:

1) ???????????????? ?????????? ???????????????????? "SAMPLE_TEAM";
2) ???????????????????????? ?????????? ???????????????????? ???????????? ?????????? ??????????????, ???????????????? "MY_TEAM" ?????? ?????????? ???????????? ????????????;
3) ???????????????? "team1.json" ?????? "team2.json" ???????? ?? ???????????????????? C:\Elsiros\controllers\referee\" 
	?????????? ???????????? ???????????? ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," ?????????????? ""robotStartCmd": "MY_TEAM/main_pb.py","
4) ?????????????????? ???????? ?? ??????????????????, ?????? ?????????????????? ???????????? ??????????????????;
5) ???????????????????? ???????????????????? ???????? "team1.json" or "team2.json" ?? ???????????????????? "MY_TEAM".
6) ?????????????????????????? ???????????????????? "MY_TEAM" ?? zip ??????????;
7) ?????????????????? "MY_TEAM.zip" ?? ???????????????????????????? ???????????????? ????????????;
8) ?????????????? ???????????? ???? ???????? "MY_TEAM.zip" ?? ???????????????? ?????????????? ???????????????????????? ????????????????????????.