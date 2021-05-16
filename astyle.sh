#!/bin/bash

astyle -r --exclude=src/lib --style=linux --suffix=none --indent=tab=8  *.cpp *.hpp
