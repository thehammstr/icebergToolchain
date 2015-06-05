#!/bin/bash

cmake .  
make  
./solveIceberg --input=../DATA/Soquel20121031/state_and_ranges.csv --links=../DATA/Soquel20121031/recordedLinks.csv
#./solveIceberg --input=../DATA/IcebergSimulation/SimulatedMultiswathData.csv --links=../DATA/IcebergSimulation/recordedLinks.csv
#./solveIceberg --input=../DATA/RealIcebergSimulation/SimulatedMultiswathDataRealIceberg1.csv --links=../DATA/RealIcebergSimulation/recordedLinks.csv

#./solveIceberg --input=../DATA/RealIcebergSimulation/SimulatedMultiswathDataRealIceberg1_50m.csv --links=../DATA/RealIcebergSimulation/recordedLinks_50m.csv

#./solveIceberg --input=../DATA/IcebergSimulation/OnlyImagenex/SimulatedMultiswathDataImagenexForHeading.csv --links=../DATA/IcebergSimulation/OnlyImagenex/recordedLinks.csv

python ../Generic/pcd2xyz.py output_cloud.pcd

python plotSolution.py

