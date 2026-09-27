@echo off
rem Builds and runs the host tests with MSVC. Usage: tests\host\run_tests.bat
setlocal
set "PATH=%PATH%;C:\Program Files (x86)\Microsoft Visual Studio\Installer"
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 2
cd /d "%~dp0"
if not exist build mkdir build
cl /nologo /std:c++17 /EHsc /W3 /utf-8 /Fe:build\host_tests.exe /Fo:build\ ^
  host_tests.cpp ..\..\src\web_bridge.cpp ..\..\src\status_json.cpp ^
  ..\..\src\net_rules.cpp ..\..\src\loop\convergence_monitor.cpp ^
  ..\..\src\loop\adjust_controller.cpp ..\..\src\loop\platform_sim.cpp ^
  ..\..\src\loop\alignment_loop.cpp ..\..\src\loop\loop_status_json.cpp ^
  ..\..\src\json_util.cpp ..\..\src\device_config.cpp ..\..\src\page_json.cpp ^
  ..\..\src\asiair\asiair_parser.cpp ..\..\src\asiair\pa_tracker.cpp ^
  ..\..\src\loop\calibration.cpp ..\..\src\loop\calibration_status.cpp ^
  ..\..\src\asiair\asiair_status.cpp ..\..\src\ota_rules.cpp ^
  ..\..\src\motion\backlash_planner.cpp ..\..\src\nina_bridge.cpp || exit /b 1
build\host_tests.exe
