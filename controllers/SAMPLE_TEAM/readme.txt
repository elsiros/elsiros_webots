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

Подготовка файлов для экспорта на соревнования.
Пошаговая инструкция.

Если вас беспокоит утечка вашего исходного кода через организаторов соревнований, то вы можете скомпилировать
ваш код в бинарный с помощью nuitka. Скомпилированный код не содержит исходного кода. Скомпилированный бинарный
 код работает быстрее, чем исходный код через интерпретатор Python. Для использования компилятора nuitka
 установите его на вашу систему, использовав следующую ссылку:
https://nuitka.net/pages/download.html
Перед установкой проверьте версию вашего Питона и выберите правильный инсталляторю
После успешной установки nuitka выполните шаги ниже: 

1) Создайте копию директории "SAMPLE_TEAM";
2) Переименуйте новую директорию именем вашей команды, например "MY_TEAM" или любым другим именем;
3) Запустите консоль Операционной Системы командой "cmd";
4) Измените директорию в консоли командой: "cd C:\Elsiros\controllers\MY_TEAM";
5) Наберите команду:"python -m nuitka --follow-imports --standalone --onefile main_pb.py";
6) В результате компиляции должен появиться файл: "main_pb.exe";
7) измените "team1.json" или "team2.json" файл в директории C:\Elsiros\controllers\referee\" 
	путем замены строки ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," строкой ""robotStartCmd": "MY_TEAM/main_pb.exe","
8) запустите игру и убедитесь, что программа играет нормально;
9) удалите все файлы "*.py" в директории "MY_TEAM" сохранив все остальные файлы и директории;
10) удалите директории: "main_pb.onefile-build", "main_pb.dist", "main_pb.build"
11) удалите все директории с именем "__pycache__";
12) скопируйте измененный файл "team1.json" or "team2.json" в директорию "MY_TEAM".
13) заархивируйте директорию "MY_TEAM" в zip архив;
14) выгрузите "MY_TEAM.zip" в предпочитаемый облачный сервер;
15) вышлите ссылку на файл "MY_TEAM.zip" в облачном сервере организатору соревнований.
16) сформированный архив должен содержать или "*.exe" или "*.json" файлы

Усли вас не беспокоит утечка вашего исходного кода через организаторов соревнований, тогда подготовте файлы выполнив
следующие шаги:

1) Создайте копию директории "SAMPLE_TEAM";
2) Переименуйте новую директорию именем вашей команды, например "MY_TEAM" или любым другим именем;
3) измените "team1.json" или "team2.json" файл в директории C:\Elsiros\controllers\referee\" 
	путем замены строки ""robotStartCmd": "SAMPLE_TEAM/main_pb.py"," строкой ""robotStartCmd": "MY_TEAM/main_pb.py","
4) запустите игру и убедитесь, что программа играет нормально;
5) скопируйте измененный файл "team1.json" or "team2.json" в директорию "MY_TEAM".
6) заархивируйте директорию "MY_TEAM" в zip архив;
7) выгрузите "MY_TEAM.zip" в предпочитаемый облачный сервер;
8) вышлите ссылку на файл "MY_TEAM.zip" в облачном сервере организатору соревнований.