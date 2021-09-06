@echo off
start /b python main_pb.py 10001 > output10001.txt 2<&1
start /b python main_pb.py 10002 >> output10002.txt
start /b python main_pb.py 10021 >> output10021.txt
start /b python main_pb.py 10022 >> output10022.txt