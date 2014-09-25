#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
using namespace std;

#include "deg2utm.h"

int main(){


double lat = 9.;
double lon = 4.;
double northing;
double easting;
int zone;
LLtoUTM(1,lat,lon,&northing,&easting,&zone);

cout<< lat << lon << northing << easting << zone <<endl;

}
