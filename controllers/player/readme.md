How to compile on Windows:  
1. Install Visual Studio 2019 community edition
3. Open player.sln with Visual Studio, do Ctrl-B (Build->Build player)    
messages.proto will be compiled automaticaly with protobuf/lib/protoc as unix makefile does. Use compile_proto.bat o do it manually if needed.
4. If debug is needed - run webots with player controller and do Debug->Attach to process->player.exe  