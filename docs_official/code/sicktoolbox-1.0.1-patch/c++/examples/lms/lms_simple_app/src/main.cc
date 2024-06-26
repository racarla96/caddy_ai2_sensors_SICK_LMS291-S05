/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 2xx driver.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <stdlib.h>
#include <string>
#include <iostream>
#include <sicklms-1.0/SickLMS.hh>
#include <chrono>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[])
{
  
  string device_str;                      
  SickLMS::sick_lms_baud_t desired_baud = SickLMS::SICK_BAUD_38400;
  
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::duration<double> elapsed_seconds;
  std::chrono::duration<double> duration[75];

  unsigned int values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickLMS.hh
  unsigned int num_values = 0;                                   // Holds the number of measurements returned

  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms_simple_app PATH [BAUD RATE]" << endl
	 << "Ex: lms_simple_app /dev/ttyUSB0 9600" << endl;
    return -1;
  }

  /* Only device path is given */
  if (argc == 2) {
    device_str = argv[1];
  }

  /* Device path and baud are given */
  if (argc == 3) {    
    device_str = argv[1];
    if ((desired_baud = SickLMS::StringToSickBaud(argv[2])) == SickLMS::SICK_BAUD_UNKNOWN) {
      cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
      return -1;
    }
  }

  /*
   * Instantiate an instance
   */
  SickLMS sick_lms(device_str);

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms.Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }

  /*
   * Acquire a few scans from the Sick LMS
   */
  try {
    start = std::chrono::system_clock::now();
    for (unsigned int i=0; i < 75; i++) {
      	sick_lms.GetSickScan(values,num_values);
      //cout << "\t  Num. Values: " << num_values << endl;
      /*
	    for (unsigned int j=0; j < SickLMS::SICK_MAX_NUM_MEASUREMENTS; j++) {
	      cout << " " << values[j];      
	    }
	      cout << endl;
	      */     
	end = std::chrono::system_clock::now();
    	elapsed_seconds = end - start;
    	duration[i] = elapsed_seconds;
	start = end;
	//cout << elapsed_seconds.count() << endl;
    }
    for (unsigned int i=0; i < 75; i++) {
	cout << duration[i].count() << endl;
    }	
  }

  /* Catch anything else and exit */ 
  catch(...) {
    cerr << "An error occurred!" << endl;
  }

  /*
   * Uninitialize the device
   */
  try {
    sick_lms.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}
    
