/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2018 Corey Stotts

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyAirspyHF.hpp"
#include <SoapySDR/Registry.hpp>
#include <cstdlib> //malloc
#include <algorithm>

static std::vector<SoapySDR::Kwargs> findAirspyHF(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;
    
    airspyhf_lib_version_t asVersion;
    airspyhf_lib_version(&asVersion);
    
    // SoapySDR_setLogLevel(SOAPY_SDR_DEBUG);
    
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AirSpyHF Lib v%d.%d rev %d", asVersion.major_version, asVersion.minor_version, asVersion.revision);

    uint64_t serials[MAX_DEVICES];
    int count = airspyhf_list_devices(serials, MAX_DEVICES);
    if (count == AIRSPYHF_ERROR) {
	    SoapySDR_logf(SOAPY_SDR_ERROR, "libairspyhf error listing devices");
    }
    else {
    	SoapySDR_logf(SOAPY_SDR_DEBUG, "%d AirSpy boards found.", count);
    }
    
    int devId = 0;
    
    for (int i = 0; i < count; i++) {
        std::stringstream serialstr;
        
        serialstr.str("");
        serialstr << std::hex << serials[i];
        
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Serial %s", serialstr.str().c_str());        

        SoapySDR::Kwargs soapyInfo;

        soapyInfo["device_id"] = std::to_string(devId);
        soapyInfo["label"] = "AirSpy HF+ [" + serialstr.str() + "]";
        soapyInfo["serial"] = serialstr.str();
        devId++;
                
        // if (args.count("serial") != 0) {
        //     if (args.at("serial") != soapyInfo.at("serial")) {
        //         continue;
        //     }
        //     SoapySDR_logf(SOAPY_SDR_DEBUG, "Found device by serial %s", soapyInfo.at("serial").c_str());
        // } else
        if (args.count("device_id") != 0) {
            if (args.at("device_id") != soapyInfo.at("device_id")) {
                continue;
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Found device by device_id %s", soapyInfo.at("device_id").c_str());
        }
        
        results.push_back(soapyInfo);
    }
   
    return results;
}

static SoapySDR::Device *makeAirspyHF(const SoapySDR::Kwargs &args)
{
    return new SoapyAirspyHF(args);
}

static SoapySDR::Registry registerAirspyHF("airspyhf", &findAirspyHF, &makeAirspyHF, SOAPY_SDR_ABI_VERSION);
