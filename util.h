#ifndef UTIL_H
#define UTIL_H
#include <math.h>

#define highLimit(v, h) (v > h ? h : v)
#define lowLimit(v, l) (v < l ? l : v)
#define hlLimit(v, h, l) ( v > h ? h : ( v < l ? l : v ) )
#define limitVar(v, l) ( v > l ? l : ( v < -l ? -l : v ) )

#define dbc(v,l) ( ( fabs(v) > l ) ? v : 0 )

#define signSquare(x) ((x<0)? -1*x*x : x*x)

#define PI 3.14159265358979323846

#endif
