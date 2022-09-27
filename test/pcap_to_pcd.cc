/******************************************************************************
 * Copyright 2020 The Hesai Technology Authors. All Rights Reserved.
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
#include <chrono>
#include <string>
#include <thread>

#include "pandarSwiftSDK.h"

#define CALIB_FILE "../params/calibrated_correction.csv"

using namespace std::chrono_literals;

static bool keep_running = true;
static int  frame_count	 = 0;

void usage(char *prog) {
	std::cout << "Usage: " << prog << " --help OR " << prog << " <pcap-file> [<calibration_file>]\n\n";
	std::cout << "\tDefault calibration file is " << CALIB_FILE << std::endl;
}

void gpsCallback(double timestamp) {}

void writePCDthread(std::string fname, boost::shared_ptr<PPointCloud> cld) {
	pcl::PCDWriter writer;

	int r = writer.writeBinaryCompressed(fname, *cld);
	switch (r) {
	case -1:
		std::cerr << __FUNCTION__ << " general error" << std::endl;
		keep_running = false;
		return;
	case -2:
		std::cerr << __FUNCTION__ << " input cloud is too large" << std::endl;
		return;
	default:
		break;
	}
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
	if (!keep_running) {
		return;
	}

	std::stringstream fname;
	// Create a per callback file name
	fname << static_cast<unsigned long long>(timestamp * 1000) << ".pcd";

	std::thread writerThread(writePCDthread, fname.str(), cld);

	// Thread such run and die without join
	writerThread.detach();
	frame_count++;
}

void rawcallback(PandarPacketsArray *array) {}

int main(int argc, char **argv) {
	char pcap_file[250];
	char corr_file[250];
	int  prev_frame_count = -1;

	if (argc > 1) {
		strcpy(pcap_file, argv[1]);
	} else {
		std::cout << "Error! Must specify input pcap file!\n";
		usage(argv[0]);
		return -1;
	}
	if (argc > 2) {
		strcpy(corr_file, argv[2]);
	} else {
		strcpy(corr_file, CALIB_FILE);
	}

	boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
	spPandarSwiftSDK.reset(new PandarSwiftSDK(
	    std::string(""), 2368, 10110, std::string("Pandar128"), std::string(corr_file),
	    std::string("../params/Pandar128_Firetimes.csv"), std::string(pcap_file), lidarCallback, rawcallback,
	    gpsCallback, std::string(""), std::string(""), std::string(""), 0, 0, std::string("point"), false));

	std::cout << "Generating PCD files from input PCAP file " << pcap_file << " specified)\n";
	int tag_done = 0;
	while (tag_done < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		if (prev_frame_count == frame_count) {
			tag_done++;
		}
		prev_frame_count = frame_count;
	}

	spPandarSwiftSDK->stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));  // grace time for writes to complete
	std::cout << "Done!" << std::endl;
	return 0;
}
