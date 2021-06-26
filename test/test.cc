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
#include "pandarSwiftSDK.h"

#include <string>
#include <thread>

void gpsCallback(double timestamp)
{
}

void writePCDthread(std::string fname, boost::shared_ptr<PPointCloud> cld)
{
    pcl::PCDWriter writer;

    int r = writer.writeBinaryCompressed(fname, *cld);
    switch (r)
    {
    case -1:
        std::cerr << __FUNCTION__ << " general error" << std::endl;
        return;
    case -2:
        std::cerr << __FUNCTION__ << " input cloud is too large" << std::endl;
        return;
    default:
        break;
    }
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    std::stringstream fname;

    // Create a per callback file name
    fname << timestamp << ".pcd";

    // Don't know how compute intensive PCD compression is so launch
    // a thread to do the compression and file write
    std::thread writerThread(writePCDthread, fname.str(), cld);

    // Thread such run and die without join
    writerThread.detach();
}

void rawcallback(PandarPacketsArray *array)
{
}

int main(int argc, char **argv)
{
    boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
    spPandarSwiftSDK.reset(new PandarSwiftSDK(std::string("192.168.1.201"), 2368, 10110, std::string("Pandar128"),
                                              std::string("../params/Pandar128_Correction.csv"),
                                              std::string("../params/Pandar128_Firetimes.csv"),
                                              std::string(""), lidarCallback, rawcallback, gpsCallback,
                                              std::string(""),
                                              std::string(""),
                                              std::string(""),
                                              0, 0, std::string("both_point_raw"), false));
    while (true)
    {
        sleep(100);
    }
    return 0;
}