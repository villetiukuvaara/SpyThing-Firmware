/*
 * gpx.c
 *
 *  Created on: Jan 6, 2018
 *      Author: parallels
 */

#include "gpx.h"
#include <stdbool.h>
#include "fatfs.h"

const char GPX_START[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
"<gpx xmlns=\"http://www.topografix.com/GPX/1/1\""
" creator=\"SpyThing\" version=\"1.1\""
" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
"\t<trk>\n";

const char GPX_END[] = "\t</trk>\n</gpx>\n";

bool gpx_start(FIL* file)
{
	FRESULT result;
	UINT btw, bw;
	btw = strlen(GPX_START);
	result = f_write(file, GPX_START, btw, &bw);
	if(result != FR_OK || btw != bw) return false;
	else return true;
}

bool gpx_trkseg_start(FIL* file)
{
	FRESULT result;
	UINT btw, bw;
	char buff[] = "\t\t<trkseg>\n";
	btw = strlen(buff);
	result = f_write(file, buff, btw, &bw);
	if(result != FR_OK || btw != bw) return false;
	else return true;
}

bool gpx_trkseg_end(FIL* file)
{
	FRESULT result;
	UINT btw, bw;
	char buff[] = "\t\t</trkseg>\n";
	btw = strlen(buff);
	result = f_write(file, buff, btw, &bw);
	if(result != FR_OK || btw != bw) return false;
	else return true;
}

bool gpx_append_trkpt(FIL* file, gps_sol_t* sol)
{
	FRESULT result;
	UINT btw, bw;
	char buff[128];

	int bytes = sprintf(buff, "\t\t\t<trkpt lat=\"%.6f\" lon=\"%.6f\">"
        "<ele>%.2f</ele>"
        "<time>%i-%02i-%02iT%02i:%02i:%02iZ</time>"
		"</trkpt>\n",
		(double)(sol->lat)*1e-7, (double)(sol->lon)*1e-7,
		(double)(sol->hMSL)/1000,
		sol->year, sol->month, sol->day,
		sol->hour, sol->min, sol->sec);
	if(bytes < 0) return false;
	else btw = bytes;
	result = f_write(file, buff, btw, &bw);
	if(result != FR_OK || btw != bw) return false;
	else return true;
}


bool gpx_end(FIL* file)
{
	FRESULT result;
	UINT btw, bw;
	btw = strlen(GPX_END);
	result = f_write(file, GPX_END, btw, &bw);
	if(result != FR_OK || btw != bw) return false;
	else return true;
}
