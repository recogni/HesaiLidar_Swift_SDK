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

using namespace std::chrono_literals;

static bool keep_running = true;

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

	// Don't know how compute intensive PCD compression is so launch
	// a thread to do the compression and file write
	std::thread writerThread(writePCDthread, fname.str(), cld);

	// Thread such run and die without join
	writerThread.detach();
}

void rawcallback(PandarPacketsArray *array) {
}

int main(int argc, char **argv) {
	auto capms = 10000ms;  // By default capture for 10 seconds

	if (argc > 1) {
		capms = std::chrono::milliseconds(static_cast<long long>(1000 * atof(argv[1])));
	}

	boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
	spPandarSwiftSDK.reset(new PandarSwiftSDK(
	    std::string("192.168.1.201"), 2368, 10110, std::string("Pandar128"),
	    std::string("../params/Pandar128_Correction.csv"), std::string("../params/Pandar128_Firetimes.csv"),
	    std::string(""), lidarCallback, rawcallback, gpsCallback, std::string(""), std::string(""), std::string(""),
	    0, 0, std::string("both_point_raw"), false));

	if (capms < 0ms) {
		std::cout << "Capturing indefinitely (" << capms.count() / 1000.0 << " specified)\n";
		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	} else {
		std::cout << "Capturing for " << capms.count() / 1000.0 << " seconds\n";
		std::this_thread::sleep_for(capms);
		keep_running = false;
	}

	spPandarSwiftSDK->stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));  // grace time for writes to complete
	std::cout << "Done!" << std::endl;
	return 0;
}
