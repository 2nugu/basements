@echo off
.\build\Release\test_solver_gpu.exe > test_result.log 2>&1
type test_result.log
