@echo off
rem Geometric robot demo. Starts its own Isaac Sim window (separate from any running one).
rem Set ISAAC_SIM_DIR if Isaac Sim is not three folders above this file.
setlocal
set "HERE=%~dp0"
if "%ISAAC_SIM_DIR%"=="" set "ISAAC_SIM_DIR=%HERE%..\..\.."
set PYTHONPATH=
call "%ISAAC_SIM_DIR%\python.bat" "%HERE%demo\demo_geometric_robot.py" %*
endlocal
