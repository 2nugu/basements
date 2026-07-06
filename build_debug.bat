@echo off
cmake --build build --target test_solver_gpu --config Release > build_log.txt 2>&1
echo Done
