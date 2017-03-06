#!/bin/sh

gcc example.c -o example  -L lib_x64 -l packet -I. -l pthread -Wl,-unresolved-symbols=ignore-in-shared-libs
