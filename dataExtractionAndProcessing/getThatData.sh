#!/bin/bash

# extract mb88 files
ls -1 | grep mb88$ > tmplist
# make datalist
#mbdatalist -F-1 -I tmplist > datalist-1


mbdatalist -F-1 -I tmplist -R-121.995833/-121.9875/36.776389/36.780556 \ > wall-datalist2
mbm_plot -F-1 -I wall-datalist2 -N -C -G1
./wall-datalist.cmd 

