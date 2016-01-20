#!/bin/bash
gcc regex_split.c -o regex_split `pkg-config --libs --cflags glib-2.0`

