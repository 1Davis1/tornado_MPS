#!/usr/bin/env bash

python3 -m ipp

echo "Build with GCC"
g++ -g -std=c++17 -Wall -I. _out.c _out.cpp main.cpp
./a.out

echo "Build with Clang"
clang++ -g -std=c++17 -Wall -I. _out.c _out.cpp main.cpp
./a.out
