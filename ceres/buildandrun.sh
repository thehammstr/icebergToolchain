#!/bin/bash

cmake .  
make  
./solveIceberg --input=../DATA/Soquel20121031/state_and_ranges.csv --links=../DATA/Soquel20121031/recordedLinks_good.csv
