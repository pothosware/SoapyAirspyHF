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

#if AIRSPYHF_VER_MAJOR >= 1
#if AIRSPYHF_VER_MINOR >= 1
#if AIRSPYHF_VER_REVISION >= 2
#define HASGAINS
#endif
#endif
#endif

SoapyAirspyHF::SoapyAirspyHF(const SoapySDR::Kwargs &args)
{
    deviceId = -1;
    serial = 0;

    //sampleRate = 3000000;
    sampleRate = 768000;
    centerFrequency = 0;

    numBuffers = DEFAULT_NUM_BUFFERS;

    agcMode = 1;
    rfBias = false;
    bitPack = false;
    lnaGain=0;
    rfGain=4;
    hasgains=false;

    bufferedElems = 0;
    resetBuffer = false;
    
    streamActive = false;
    sampleRateChanged.store(false);
    
    dev = nullptr;
    bool serial_specified = false;
    std::stringstream serialstr;
    serialstr.str("");

    if (args.count("serial") != 0)
    {
        try {
            serial = std::stoull(args.at("serial"), nullptr, 16);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("serial is not a hex number");
        } catch (const std::out_of_range &) {
            throw std::runtime_error("serial value of out range");
    }
        serial_specified = true;
    }
    else if (args.count("device_id") != 0)
    {
        try {
            deviceId = std::stoi(args.at("device_id"));
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("device_id is not a number");
        } catch (const std::out_of_range &) {
            throw std::runtime_error("device_id of out range");
        }
    }
    uint64_t serials[MAX_DEVICES];
    int numDevices = airspyhf_list_devices(serials, MAX_DEVICES);
    if (numDevices == AIRSPYHF_ERROR) {
        throw std::runtime_error("Failed to retrieve list of AirspyHF devices");
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "%d AirSpyHF devices found.", numDevices);
    if (numDevices == 0) {
        throw std::runtime_error("No AirspyHF devices found");
    }

    if (serial_specified) {     // search by serial
        serialstr << std::hex << serial;
        for (int i = 0; i < numDevices; i++)
        {
            if (serials[i] == serial)
            {
                deviceId = i;
                break;
            }
        }
        if (deviceId < 0)
        {
            throw std::runtime_error("AirspyHF device with S/N " + serialstr.str() + " not found");
        }
    }
    else {                    // search by device_id
        if (deviceId < 0)
        {
            deviceId = 0;       // default if no device_id specified
        }
        else if (deviceId >= numDevices)
        {
            throw std::runtime_error("Airspy device_id out of range [0 .. " + std::to_string(numDevices-1) + "].");
        }
        serial = serials[deviceId];
        serialstr << std::hex << serial;
    }
    if (airspyhf_open_sn(&dev, serial) != AIRSPYHF_SUCCESS) {
        throw std::runtime_error("Unable to open AirspyHF device " + std::to_string(deviceId) +
            "with S/N " + serialstr.str());
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Found AirspyHF+ device: device_id = %d, serial = %16Lx", deviceId, serial);

#ifdef HASGAINS
    if (airspyhf_set_hf_att(dev,rfGain)==AIRSPYHF_SUCCESS) {
        hasgains=true;
        airspyhf_set_hf_lna(dev,lnaGain);
        airspyhf_set_hf_agc(dev,agcMode);
    }
#endif

    //apply arguments to settings when they match
    for (const auto &info : this->getSettingInfo())
    {
        const auto it = args.find(info.key);
        if (it != args.end()) this->writeSetting(it->first, it->second);
    }
}

SoapyAirspyHF::~SoapyAirspyHF(void)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    airspyhf_close(dev);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspyHF::getDriverKey(void) const
{
    return "AirspyHF";
}

std::string SoapyAirspyHF::getHardwareKey(void) const
{
    return "AirspyHF";
}

SoapySDR::Kwargs SoapyAirspyHF::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["device_id"] = std::to_string(deviceId);

    std::stringstream serialstr;
    serialstr.str("");
    serialstr << std::hex << serial;
    args["serial"] = serialstr.str();

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspyHF::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyAirspyHF::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    // TODO
}

std::string SoapyAirspyHF::getAntenna(const int direction, const size_t channel) const
{
    // eventually could change this to HF/VHF
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspyHF::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;

    if (hasgains) {
        results.push_back("LNA");
        results.push_back("RF");
    }

    return results;
}

bool SoapyAirspyHF::hasGainMode(const int direction, const size_t channel) const
{
    return true; // we have agc on/off setting or it's forced on, either way AGC is supported
}

void SoapyAirspyHF::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    //SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AGC: %s", automatic ? "Automatic" : "Manual");
    if (!hasgains) return;
    
#ifdef HASGAINS
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    airspyhf_set_hf_agc(dev,agcMode=automatic ? 1:0);
#endif
}

bool SoapyAirspyHF::getGainMode(const int direction, const size_t channel) const
{
    return agcMode ? true : false; //agc is finally not always on
}

SoapySDR::Range SoapyAirspyHF::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (!hasgains) return SoapySDR::Range(0.0, 0.0);
    if (name == "LNA") return SoapySDR::Range(0,6,6);
    return SoapySDR::Range(-48.0,0,6);
}

double SoapyAirspyHF::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (!hasgains) return 0.0;
    if (name=="LNA") return lnaGain*6.0;
    return -(int)rfGain*6.0;
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if (!hasgains) return;
#ifdef HASGAINS
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    if (name == "LNA") {
        lnaGain = value>=3.0 ? 1 : 0;
        airspyhf_set_hf_lna(dev,lnaGain);
        return;
    }
    double newval = -value;
    if (newval<0.0) newval=0.0;
    if (newval>48.0) newval=48.0;
    rfGain=(uint8_t)(newval/6.0+0.499);
    airspyhf_set_hf_att(dev,rfGain);
#endif
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspyHF::setFrequency(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        centerFrequency = (uint32_t) frequency;
        std::lock_guard <std::mutex> lock(_general_state_mutex);
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting center freq: %d", centerFrequency);
        airspyhf_set_freq(dev, centerFrequency);
    }
}

double SoapyAirspyHF::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF")
    {
        return (double) centerFrequency;
    }

    return 0;
}

std::vector<std::string> SoapyAirspyHF::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyAirspyHF::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(9000,31000000));
	results.push_back(SoapySDR::Range(60000000,260000000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspyHF::setSampleRate(const int direction, const size_t channel, const double rate)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d", sampleRate);

    if (sampleRate != rate) {
        sampleRate = rate;
        resetBuffer = true;
        sampleRateChanged.store(true);
    }
}

double SoapyAirspyHF::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate;
}

std::vector<double> SoapyAirspyHF::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    std::lock_guard <std::mutex> lock(_general_state_mutex);

    uint32_t numRates;
	airspyhf_get_samplerates(dev, &numRates, 0);

	std::vector<uint32_t> samplerates;
    samplerates.resize(numRates);
    
	airspyhf_get_samplerates(dev, samplerates.data(), numRates);

	for (auto i: samplerates) {
        results.push_back(i);
	}

    return results;
}

void SoapyAirspyHF::setBandwidth(const int direction, const size_t channel, const double bw)
{
    SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyAirspyHF::getBandwidth(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getBandwidth(direction, channel);
}

std::vector<double> SoapyAirspyHF::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspyHF::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;
 
    return setArgs;
}

void SoapyAirspyHF::writeSetting(const std::string &key, const std::string &value)
{

}

std::string SoapyAirspyHF::readSetting(const std::string &key) const
{
    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}
