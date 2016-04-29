#!/bin/bash

IDIR="LzwInputData/"
ODIR="LzwOutputData/"

for i in {1..4};
do
	FILE1=$IDIR"compressedfile"$i".z"
	FILE2=$ODIR"recompressedfile"$i".z"
	diff $FILE1 $FILE2
done
