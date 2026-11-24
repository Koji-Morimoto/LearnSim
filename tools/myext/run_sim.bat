@echo off
rem Run a SimulationApp script with a UTF-8 console.
rem Usage: tools\myext\run_sim.bat path\to\script.py [args...]
chcp 65001 > nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
call "%~dp0..\..\python.bat" %*
