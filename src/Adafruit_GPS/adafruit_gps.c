#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "adafruit_gps.h"
#include <string.h> 
#include <stdlib.h>

uint8_t _adafruit_hour, _adafruit_minute, _adafruit_seconds, _adafruit_year, _adafruit_month, _adafruit_day;
uint16_t _adafruit_milliseconds;
float _adafruit_latitude, _adafruit_longitude;
// Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
// and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
//   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
int32_t _adafruit_latitude_fixed, _adafruit_longitude_fixed;
float _adafruit_latitudeDegrees, _adafruit_longitudeDegrees;
float _adafruit_geoidheight, _adafruit_altitude;
float _adafruit_speed, _adafruit_angle, _adafruit_magvariation, _adafruit_HDOP;
char _adafruit_lat, _adafruit_lon, _adafruit_mag;
bool _adafruit_fix;
uint8_t _adafruit_fixquality, _adafruit_satellites;
bool _adafruit_isSleep = false;
uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

bool adafruit_gps_parse(char *nmea) {
  // do checksum check

  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);
    
    // check checksum 
    for (uint8_t i=2; i < (strlen(nmea)-4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
      return false;
    }
  }
  int32_t degree;
  long minutes;
  char degreebuff[10];
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
    // found GGA
    char *p = nmea;
    // get time
    p = (char* ) (strchr(p, ',')+1);
    float timef = atof(p);
    uint32_t time = timef;
    _adafruit_hour = time / 10000;
    _adafruit_minute = (time % 10000) / 100;
    _adafruit_seconds = (time % 100);

    _adafruit_milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      _adafruit_latitude_fixed = degree + minutes;
      _adafruit_latitude = degree / 100000 + minutes * 0.000006F;
      _adafruit_latitudeDegrees = (_adafruit_latitude-100* (int)(_adafruit_latitude/100))/60.0;
      _adafruit_latitudeDegrees += (int)(_adafruit_latitude/100);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      if (p[0] == 'S') _adafruit_latitudeDegrees *= -1.0;
      if (p[0] == 'N') _adafruit_lat = 'N';
      else if (p[0] == 'S') _adafruit_lat = 'S';
      else if (p[0] == ',') _adafruit_lat = 0;
      else return false;
    }
    
    // parse out longitude
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      _adafruit_longitude_fixed = degree + minutes;
      _adafruit_longitude = degree / 100000 + minutes * 0.000006F;
      _adafruit_longitudeDegrees = (_adafruit_longitude-100*(int)(_adafruit_longitude/100))/60.0;
      _adafruit_longitudeDegrees += (int)(_adafruit_longitude/100);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      if (p[0] == 'W') _adafruit_longitudeDegrees *= -1.0;
      if (p[0] == 'W') _adafruit_lon = 'W';
      else if (p[0] == 'E') _adafruit_lon = 'E';
      else if (p[0] == ',') _adafruit_lon = 0;
      else return false;
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_fixquality = atoi(p);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_satellites = atoi(p);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_HDOP = atof(p);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_altitude = atof(p);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_geoidheight = atof(p);
    }
    return true;
  }
  if (strstr(nmea, "$GPRMC")) {
   // found RMC
    char *p = nmea;

    // get time
    p = (char* ) (strchr(p, ',')+1);
    float timef = atof(p);
    uint32_t time = timef;
    _adafruit_hour = time / 10000;
    _adafruit_minute = (time % 10000) / 100;
    _adafruit_seconds = (time % 100);

    _adafruit_milliseconds = fmod(timef, 1.0) * 1000;

    p = (char* ) (strchr(p, ',')+1);
    // Serial.println(p);
    if (p[0] == 'A') 
      _adafruit_fix = true;
    else if (p[0] == 'V'){
      _adafruit_fix = false;
		return false;
	}
    else
      return false;

    // parse out latitude
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;
      _adafruit_latitude_fixed = degree + minutes;
      _adafruit_latitude = degree / 100000 + minutes * 0.000006F;
      _adafruit_latitudeDegrees = (_adafruit_latitude-100*(int)(_adafruit_latitude/100))/60.0;
      _adafruit_latitudeDegrees += (int)(_adafruit_latitude/100);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      if (p[0] == 'S') _adafruit_latitudeDegrees *= -1.0;
      if (p[0] == 'N') _adafruit_lat = 'N';
      else if (p[0] == 'S') _adafruit_lat = 'S';
      else if (p[0] == ',') _adafruit_lat = 0;
      else return false;
    }
    
    // parse out longitude
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      _adafruit_longitude_fixed = degree + minutes;
      _adafruit_longitude = degree / 100000 + minutes * 0.000006F;
      _adafruit_longitudeDegrees = (_adafruit_longitude-100*(int)(_adafruit_longitude/100))/60.0;
      _adafruit_longitudeDegrees += (int)(_adafruit_longitude/100);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      if (p[0] == 'W') _adafruit_longitudeDegrees *= -1.0;
      if (p[0] == 'W') _adafruit_lon = 'W';
      else if (p[0] == 'E') _adafruit_lon = 'E';
      else if (p[0] == ',') _adafruit_lon = 0;
      else return false;
    }
    // speed
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_speed = atof(p);
    }
    
    // angle
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      _adafruit_angle = atof(p);
    }
    
    p = (char* ) (strchr(p, ',')+1);
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      _adafruit_day = fulldate / 10000;
      _adafruit_month = (fulldate % 10000) / 100;
      _adafruit_year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
    return true;
  }

  return false;
}

void adafruit_gps_send_command(char * command)
{
	int len = strlen(command);
	uint8_t * com = (uint8_t *)command;
	for (int i = 0; i < len; i++){
		app_uart_put(command[i]);
	}
	app_uart_put('\r');
	app_uart_put('\n');
}
bool adafruit_gps_sleep()
{
	if (_adafruit_isSleep) {
		return false;
	}
	adafruit_gps_send_command(PMTK_STANDBY);
	_adafruit_isSleep = true;
}
bool adafruit_gps_wakeup()
{
	// config the gps module
	// remember to add \r\n at the end of the command sending to the gps module
	if (!_adafruit_isSleep) {
		return false;
	}
	char * com = "\r\n";
	uint8_t* command = (uint8_t *)com;
	for (int i = 0; i < strlen(com); i++){
		app_uart_put(command[i]);
	}
}

adafruit_position_t adafruit_get_last_position(){
	adafruit_position_t output = {_adafruit_latitudeDegrees, _adafruit_longitudeDegrees, _adafruit_altitude};
	return output;
}

bool adafruit_is_fix(){
	return _adafruit_fix;
}
