/*
 * gpx.h
 *
 *  Created on: Jan 6, 2018
 *      Author: parallels
 */

#ifndef GPX_H_
#define GPX_H_

#include <stdbool.h>
#include "fatfs.h"
#include "gps.h"

bool gpx_start(FIL* file);
bool gpx_end(FIL* file);
bool gpx_trkseg_start(FIL* file);
bool gpx_trkseg_end(FIL* file);
bool gpx_append_trkpt(FIL* file, gps_sol_t* sol);


#endif /* GPX_H_ */
