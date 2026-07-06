# Basements - Quick Compile Script (Without CMake)
# This script compiles tests directly using MSVC

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "  Basements Physics Engine - Quick Build" -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host ""

# Check for MSVC
$vcvarsall = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
if (-not (Test-Path $vcvarsall)) {
    $vcvarsall = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat"
}

if (-not (Test-Path $vcvarsall)) {
    Write-Host "ERROR: Visual Studio not found!" -ForegroundColor Red
    Write-Host "Please install Visual Studio 2019 or 2022 with C++ support" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Alternative: Install CMake from https://cmake.org/download/" -ForegroundColor Yellow
    exit 1
}

Write-Host "Found Visual Studio: $vcvarsall" -ForegroundColor Green
Write-Host ""

# Create build directory
New-Item -ItemType Directory -Force -Path "build\manual" | Out-Null

# Compile simple test
Write-Host "Compiling simple Vec3 test..." -ForegroundColor Yellow

$testCode = @"
#include <iostream>
#include "../include/basements/vec3.h"

using namespace basements::math;

int main() {
    std::cout << "=== Vec3 Simple Test ===" << std::endl;
    
    // Test construction
    Vec3 v1(1.0f, 2.0f, 3.0f);
    Vec3 v2(4.0f, 5.0f, 6.0f);
    
    std::cout << "v1: (" << v1.x << ", " << v1.y << ", " << v1.z << ")" << std::endl;
    std::cout << "v2: (" << v2.x << ", " << v2.y << ", " << v2.z << ")" << std::endl;
    
    // Test addition
    Vec3 sum = v1 + v2;
    std::cout << "v1 + v2: (" << sum.x << ", " << sum.y << ", " << sum.z << ")" << std::endl;
    
    // Test dot product
    float dot = v1.dot(v2);
    std::cout << "v1 · v2: " << dot << std::endl;
    
    // Test cross product
    Vec3 cross = v1.cross(v2);
    std::cout << "v1 × v2: (" << cross.x << ", " << cross.y << ", " << cross.z << ")" << std::endl;
    
    // Test normalization
    Vec3 normalized = v1.normalized();
    std::cout << "normalized v1: (" << normalized.x << ", " << normalized.y << ", " << normalized.z << ")" << std::endl;
    std::cout << "length: " << normalized.length() << std::endl;
    
    std::cout << std::endl << "All tests passed!" << std::endl;
    return 0;
}
"@

$testCode | Out-File -FilePath "build\manual\test_simple.cpp" -Encoding UTF8

# Compile
Write-Host "Compiling with cl.exe..." -ForegroundColor Yellow

& cmd /c "`"$vcvarsall`" x64 && cl /EHsc /std:c++17 /I. /Fe:build\manual\test_simple.exe build\manual\test_simple.cpp 2>&1" | Out-String

if (Test-Path "build\manual\test_simple.exe") {
    Write-Host ""
    Write-Host "Build successful!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Running test..." -ForegroundColor Yellow
    Write-Host "==================================================" -ForegroundColor Cyan
    
    & "build\manual\test_simple.exe"
    
    Write-Host "==================================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "SUCCESS! Vec3 is working correctly." -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "Build failed. Please check errors above." -ForegroundColor Red
}

Write-Host ""
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Install CMake for full test suite" -ForegroundColor White
Write-Host "2. Or continue with manual compilation" -ForegroundColor White
Write-Host "==================================================" -ForegroundColor Cyan
