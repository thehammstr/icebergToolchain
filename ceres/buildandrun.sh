#!/bin/bash

cmake .  
make  
./solveIceberg --input=../DATA/Soquel20121031/state_and_ranges.csv --links=../DATA/Soquel20121031/recordedLinks.csv
#./solveIceberg --input=../DATA/IcebergSimulation/SimulatedMultiswathData.csv --links=../DATA/IcebergSimulation/recordedLinks.csv
