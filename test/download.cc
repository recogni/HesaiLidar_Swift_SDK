/******************************************************************************
 * Copyright 2022 Recogni Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <fstream>
#include <iostream>
#include <string>

#include "pandarSwiftSDK.h"

void usage(char *prog) {
	std::cout << "Usage: " << prog << " [-h] | [<ip-addr> [<output_file_name]]\n\n";
	std::cout << "\tDefault ip addr is 192.168.1.201 and \n\tdefault output file is correction.csv" << std::endl;
}

// This function is used only to test the parsing of the loaded correction data from the lidar
static int loadCorrectionString(std::string correction_content) {
	std::istringstream ifs(correction_content);
	std::string	   line;
	if (std::getline(ifs, line)) {	// first line "Laser id,Elevation,Azimuth"
		printf("Parsing Lidar Correction...\n");
	}
	float pitchList[PANDAR128_LASER_NUM];
	float azimuthList[PANDAR128_LASER_NUM];
	int   lineCounter = 0;
	while (std::getline(ifs, line)) {
		if (line.length() < strlen("1,1,1")) {
			return -1;
		} else {
			lineCounter++;
		}
		float		  elev, azimuth;
		int		  lineId = 0;
		std::stringstream ss(line);
		std::string	  subline;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> lineId;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> elev;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> azimuth;
		if (lineId != lineCounter) {
			printf("laser id error %d %d\n", lineId, lineCounter);
			return -1;
		}
		pitchList[lineId - 1]	= elev;
		azimuthList[lineId - 1] = azimuth;
	}
	return 0;
}

static std::string loadCorrectionFile(std::string ipAddr) {
	bool	    loadCorrectionFileSuccess = false;
	void	     *m_pTcpCommandClient;
	int	    ret;
	std::string correntionString;

	m_pTcpCommandClient = TcpCommandClientNew(ipAddr.c_str(), PANDARSDK_TCP_COMMAND_PORT);
	if (m_pTcpCommandClient != NULL) {
		char    *buffer = NULL;
		uint32_t len	= 0;
		ret		= TcpCommandGetLidarCalibration(m_pTcpCommandClient, &buffer, &len);
		if (ret == 0 && buffer) {
			printf("Loading correction file from lidar now!\n");
			correntionString = std::string(buffer);
			ret		 = loadCorrectionString(correntionString);
			if (ret != 0) {
				printf("Lidar Correction Parsing Error: \n");
			} else {
				loadCorrectionFileSuccess = true;
				printf("Parsing Lidar Correction Successful!\n");
			}
			free(buffer);
		} else {
			printf("Loading lidar calibration failed\n");
			correntionString = "";
		}
	}
	return correntionString;
}

int main(int argc, char **argv) {
	char	    correction_file[250];
	char	    opt1[16];
	std::string ipAddr;

	if (argc > 1) {
		strcpy(opt1, argv[1]);
		if (!strcmp(opt1, "-h") || !strcmp(opt1, "--help")) {
			usage(argv[0]);
			return 0;
		}
		ipAddr = opt1;
	} else {
		ipAddr = "192.168.1.201";
	}

	if (argc == 3) {
		strcpy(correction_file, argv[2]);
	} else {
		strcpy(correction_file, "./corrections.csv");
	}

	std::string   correction = loadCorrectionFile(ipAddr);
	std::ofstream ofile(correction_file);
	ofile << correction;
	ofile.close();
	return 0;
}
