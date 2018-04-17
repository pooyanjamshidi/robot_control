#!/usr/bin/env bash

FILE=~/cp1/current-task-finished
DONEMSG="DONE"

if [ -e $FILE ]
then
    echo $DONEMSG >> $FILE
else
    echo "file does not exists"
fi
