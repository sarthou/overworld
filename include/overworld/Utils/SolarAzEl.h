#ifndef OWDS_SOLARAZEL_H
#define OWDS_SOLARAZEL_H

#include <ctime>

void solarAzEl(time_t utc_time_point, double Lat, double Lon, double Alt, double* Az, double* El);

#endif // OWDS_SOLARAZEL_H