How to compile on Windows:  
1. Install Visual Studio 2019 community edition
2. If needed - compile new messages.proto with compile_proto.bat   
Result messages.pb.cc and messages.pb.h will be in /messages dir 
3. Open player.sln with Visual Studio, do Ctrl-B (Build->Build player)  
4. If debug is needed - run webots with player controller and do Debug->Attach to process->player.exe  