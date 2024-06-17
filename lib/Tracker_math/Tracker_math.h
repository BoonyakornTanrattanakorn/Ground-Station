#ifndef TRACKER_MATH_H
#define TRACKER_MATH_H
#include <math.h>

const double a = 6378137.0, b = 6356752.3, a2 = a*a, b2 = b*b; // equatorial and polar radius in meters

float cal_el(double lat_rx, double long_rx, double lat_tx, double long_tx, double alt_rx, double alt_tx){
  // all lat and long is in degrees, altitude in meters, return elevation in degrees
  double A = lat_rx*deg2rad, B = long_rx*deg2rad, C = lat_tx*deg2rad, D = long_tx*deg2rad, R = WGS84_radius(lat_rx)+alt_rx;
  return rad2deg * atan( ( cos(A-C)*cos(B-D) - (R)/(R+alt_tx) ) / sqrt(1 - sq((cos(A-C)*cos(B-D))) ) );
}

float cal_az(double lat_rx, double long_rx, double lat_tx, double long_tx){
  // all lat and long in degrees, return azimuth in degrees (0 - 360 north = 0)
  if(lat_tx >= lat_rx){
    if(long_tx >= long_rx){
      return RAD_TO_DEG * atan( tan( (long_rx-long_tx)*DEG_TO_RAD ) / sin( (lat_rx-lat_tx)*DEG_TO_RAD ) );
    }else{
      return 360 + RAD_TO_DEG * atan( tan( (long_rx-long_tx)*DEG_TO_RAD ) / sin( (lat_rx-lat_tx)*DEG_TO_RAD ) );
    }
  }else{
    if(long_tx >= long_rx){
      return 180 + RAD_TO_DEG * atan( tan( (long_rx-long_tx)*DEG_TO_RAD ) / sin( (lat_rx-lat_tx)*DEG_TO_RAD ) );
    }else{
      return 180 + RAD_TO_DEG * atan( tan( (long_rx-long_tx)*DEG_TO_RAD ) / sin( (lat_rx-lat_tx)*DEG_TO_RAD ) );
    }
  }
}

double WGS84_radius(double lat_rx){
  // return earth radius in meters from latitude in degrees
  double B = lat_rx*DEG_TO_RAD;
  return sqrt( (sq(a2*cos(B)) + sq(b2*sin(B))) / (sq(a*cos(B)) + sq(b*sin(B))) );
}

#endif