#!/bin/sh

gcc example.c -o example  -L lib_x64 -l packet -I. -l pthread -Wl,-unresolved-symbols=ignore-in-shared-libs
gcc example_pu_connections.c -o example_pu_connections  -L lib_x64 -l packet -I. -l pthread -Wl,-unresolved-symbols=ignore-in-shared-libs
gcc test_phase1.c -o test_phase1  -L lib_x64 -l packet -I. -l pthread -Wl,-unresolved-symbols=ignore-in-shared-libs
