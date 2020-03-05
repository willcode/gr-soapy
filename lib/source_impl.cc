/* -*- c++ -*- */
/*
 * gr-soapy: Soapy SDR Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2018, 2019, 2020
 *  Libre Space Foundation <http://libre.space>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "source_impl.h"
#include <SoapySDR/Formats.h>

const pmt::pmt_t CMD_CHAN_KEY = pmt::mp("chan");
const pmt::pmt_t CMD_FREQ_KEY = pmt::mp("freq");
const pmt::pmt_t CMD_GAIN_KEY = pmt::mp("gain");
const pmt::pmt_t CMD_ANTENNA_KEY = pmt::mp("antenna");
const pmt::pmt_t CMD_RATE_KEY = pmt::mp("samp_rate");
const pmt::pmt_t CMD_BW_KEY = pmt::mp("bw");

namespace gr {
namespace soapy {
source::sptr
source::make(size_t nchan, const std::string &device,
             const std::string &args, double sampling_rate,
             const std::string &type)
{
  return gnuradio::get_initial_sptr(
           new source_impl(nchan, device, args, sampling_rate, type));
}

/*
 * The private constructor
 */
source_impl::source_impl(size_t nchan, const std::string &device,
                         const std::string &args, double sampling_rate,
                         const std::string &type) :
  gr::sync_block("soapy::source", gr::io_signature::make(0, 0, 0),
                 args_to_io_sig(type, nchan)),
  /*
   * Swig does not guarantee C++ destructors are called from python,
   *  so recommends against relying on them for cleanup.
   */
  d_stopped(true),
  d_mtu(0),
  d_sampling_rate(sampling_rate),
  d_nchan(nchan)
{

  /* "serial" device argument needs special handling. */
  std::string dev_str = device;
  SoapySDR::Kwargs dev_args = SoapySDR::KwargsFromString(args);
  const SoapySDR::Kwargs::const_iterator iter = dev_args.find("serial");
  if (iter != dev_args.end()) {
    dev_str += ",serial=" + iter->second;
    dev_args.erase(iter->first);
  }

  std::string stype;
  if (type == "fc32") {
    stype = SOAPY_SDR_CF32;
  }
  else if (type == "sc16") {
    stype = SOAPY_SDR_CS16;
  }
  else if (type == "sc8") {
    stype = SOAPY_SDR_CS8;
  }
  else {
    std::string msg = name() + ": Invalid IO type";
    throw std::invalid_argument(msg);
  }

  d_device = SoapySDR::Device::make(dev_str);
  d_stopped = false;

  if (d_nchan > d_device->getNumChannels(SOAPY_SDR_RX)) {
    std::string msg = name() +  ": Unsupported number of channels. Only  "
                      + std::to_string(d_device->getNumChannels(SOAPY_SDR_RX))
                      + " channels available.";
    throw std::invalid_argument(msg);
  }

  std::vector<size_t> channs;
  channs.resize(d_nchan);
  for (size_t i = 0; i < d_nchan; i++) {
    channs[i] = i;

    SoapySDR::RangeList sampRange = d_device->getSampleRateRange(SOAPY_SDR_RX, i);

    double minRate = sampRange.front().minimum();
    double maxRate = sampRange.back().maximum();

    // for some reason the airspy provides them backwards
    if (minRate > maxRate) {
      std::swap(minRate, maxRate);
    }

    if ((d_sampling_rate < minRate) || (d_sampling_rate > maxRate)) {
      std::string msg = name() + ": Unsupported sample rate.  Rate must be between "
                        + std::to_string(minRate)
                        + " and " + std::to_string(maxRate);
      throw std::invalid_argument(msg);
    }
    set_sample_rate(i, d_sampling_rate);
  }

  /* Apply to all streams the supported args */
  SoapySDR::ArgInfoList supported_args =
    d_device->getStreamArgsInfo(SOAPY_SDR_RX, 0);
  SoapySDR::Kwargs stream_args;
  for (const SoapySDR::ArgInfo &i : supported_args) {
    const SoapySDR::Kwargs::const_iterator iter = dev_args.find(i.key);
    if (iter != dev_args.end()) {
      stream_args[iter->first] = iter->second;
      /*
       * This was a stream argument. Erase it so it cannot be applied later
       * as device argument
       */
      dev_args.erase(i.key);
    }
  }

  d_stream = d_device->setupStream(SOAPY_SDR_RX, stype, channs, stream_args);
  d_mtu = d_device->getStreamMTU(d_stream);

  /*
   * Apply device settings to all enabled channels.
   * These should be any settings that were not a stream argument
   */
  for (size_t chan : channs) {
    for (const std::pair<std::string, std::string> &iter : dev_args) {
      d_device->writeSetting(SOAPY_SDR_RX, chan, iter.first, iter.second);
    }
  }

  message_port_register_in(pmt::mp("command"));
  set_msg_handler(
    pmt::mp("command"),
    boost::bind(&source_impl::msg_handler_command, this, _1));

  register_msg_cmd_handler(
    CMD_FREQ_KEY,
    boost::bind(&source_impl::cmd_handler_frequency, this, _1, _2));
  register_msg_cmd_handler(
    CMD_GAIN_KEY,
    boost::bind(&source_impl::cmd_handler_gain, this, _1, _2));
  register_msg_cmd_handler(
    CMD_RATE_KEY,
    boost::bind(&source_impl::cmd_handler_samp_rate, this, _1, _2));
  register_msg_cmd_handler(
    CMD_BW_KEY, boost::bind(&source_impl::cmd_handler_bw, this, _1, _2));
  register_msg_cmd_handler(
    CMD_ANTENNA_KEY,
    boost::bind(&source_impl::cmd_handler_antenna, this, _1, _2));

  /* GNU Radio stream engine is very efficient when the buffers are a power of 2*/
  set_output_multiple(1024);

  /* This limits each work invocation to MTU transfers */
  set_max_noutput_items(d_mtu);
}

bool
source_impl::start()
{
  d_device->activateStream(d_stream);
  return true;
}

bool
source_impl::stop()
{
  if (!d_stopped) {
    d_device->closeStream(d_stream);
    SoapySDR::Device::unmake(d_device);
    d_stopped = true;
  }
  return true;
}

source_impl::~source_impl()
{
  /*
   * Guarded with the d_stopped flag.
   * if \ref stop() has not yet called for releasing hardware etc,
   * it will be called by this destructor.
   *
   * If the scheduler has already called \ref stop() calling it again should have
   * no impact.
   */
  stop();
}

void
source_impl::register_msg_cmd_handler(const pmt::pmt_t &cmd,
                                      cmd_handler_t handler)
{
  d_cmd_handlers[cmd] = handler;
}

bool
source_impl::hasDCOffset(int channel)
{
  return d_device->hasDCOffset(SOAPY_SDR_RX, channel);
}

bool
source_impl::hasIQBalance(int channel)
{
  return d_device->hasIQBalance(SOAPY_SDR_RX, channel);
}

bool
source_impl::hasFrequencyCorrection(int channel)
{
  return d_device->hasFrequencyCorrection(SOAPY_SDR_RX, channel);
}

void
source_impl::set_frequency(size_t channel, double frequency)
{
  if (channel >= d_nchan) {
    return;
  }

  d_device->setFrequency(SOAPY_SDR_RX, channel, frequency);
  d_frequency = d_device->getFrequency(SOAPY_SDR_RX, channel);
}

void
source_impl::set_frequency(size_t channel, const std::string &name,
                           double frequency)
{
  if (channel >= d_nchan) {
    return;
  }

  d_device->setFrequency(SOAPY_SDR_RX, channel, name, frequency);
}

bool
source_impl::gain_available(size_t channel, const std::string &name)
{
  std::vector<std::string> gains = d_device->listGains(SOAPY_SDR_RX, channel);

  if (std::find(gains.begin(), gains.end(), name) != gains.end()) {
    return true;
  }
  return false;
}

void
source_impl::set_gain(size_t channel, float gain)
{
  if (channel >= d_nchan) {
    return;
  }
  SoapySDR::Range rGain = d_device->getGainRange(SOAPY_SDR_RX, channel);

  if (gain < rGain.minimum() || gain > rGain.maximum()) {
    GR_LOG_WARN(d_logger,
                boost::format("Gain out of range: %d <= gain <= %d") %
                rGain.minimum() % rGain.maximum());
    return;
  }

  d_device->setGain(SOAPY_SDR_RX, channel, gain);
  d_gain = d_device->getGain(SOAPY_SDR_RX, channel);
}

void
source_impl::set_gain(size_t channel, const std::string name, float gain)
{
  /*
   * This setter is for manual mode gain. There is a known limitation of the
   * GRC and the yaml runtime evaluation, so we need to skip this setting
   * if the mode is auto gain
   */
  if (channel >= d_nchan) {
    return;
  }

  if (!gain_available(channel, name)) {
    GR_LOG_WARN(d_logger,
                boost::format("Unknown %s gain setting "
                              "for channel %zu") % name % channel);
    return;
  }

  SoapySDR::Range rGain = d_device->getGainRange(SOAPY_SDR_RX, channel, name);

  if (gain < rGain.minimum() || gain > rGain.maximum()) {
    GR_LOG_WARN(d_logger,
                boost::format("Gain %s out of range: %d <= gain <= %d")
                % name % rGain.minimum() % rGain.maximum());
  }

  d_device->setGain(SOAPY_SDR_RX, channel, name, gain);
}

void
source_impl::set_gain_mode(size_t channel, bool gain_auto_mode)
{
  if (channel >= d_nchan) {
    return;
  }

  d_device->setGainMode(SOAPY_SDR_RX, channel, gain_auto_mode);
}

void
source_impl::set_sample_rate(size_t channel, double sample_rate)
{
  if (channel >= d_nchan) {
    return;
  }

  d_device->setSampleRate(SOAPY_SDR_RX, channel, sample_rate);
  d_sampling_rate = sample_rate;
}

std::vector<std::string>
source_impl::listAntennas(int channel)
{
  if ((size_t)channel >= d_nchan) {
    return std::vector<std::string>();
  }

  return d_device->listAntennas(SOAPY_SDR_RX, channel);
}

void
source_impl::set_bandwidth(size_t channel, double bandwidth)
{
  if (channel >= d_nchan) {
    return;
  }

  d_device->setBandwidth(SOAPY_SDR_RX, channel, bandwidth);
  d_bandwidth = bandwidth;
}

void
source_impl::set_antenna(const size_t channel, const std::string &name)
{
  if (channel >= d_nchan) {
    return;
  }

  std::vector<std::string> antennaList = d_device->listAntennas(SOAPY_SDR_RX,
                                         channel);

  if (antennaList.size() > 0) {
    if (std::find(antennaList.begin(), antennaList.end(),
                  name) == antennaList.end()) {
      GR_LOG_WARN(d_logger,
                  boost::format("Antenna name %s not supported.") % name);
      return;
    }
  }

  d_device->setAntenna(SOAPY_SDR_RX, channel, name);
  d_antenna = name;
}

void
source_impl::set_dc_offset(size_t channel, gr_complexd dc_offset,
                           bool dc_offset_auto_mode)
{
  if (channel >= d_nchan) {
    return;
  }

  if (!hasDCOffset(channel)) {
    return;
  }

  /* If DC Correction is supported but automatic mode is not set DC correction */
  if (!dc_offset_auto_mode
      && d_device->hasDCOffset(SOAPY_SDR_RX, channel)) {
    d_device->setDCOffset(SOAPY_SDR_RX, channel, dc_offset);
    d_dc_offset = dc_offset;
  }
}

void
source_impl::set_dc_offset_mode(size_t channel, bool dc_offset_auto_mode)
{
  if (channel >= d_nchan) {
    return;
  }

  if (!hasDCOffset(channel)) {
    return;
  }

  /* If user specifies automatic DC Correction and is supported activate it */
  if (dc_offset_auto_mode
      && d_device->hasDCOffsetMode(SOAPY_SDR_RX, channel)) {
    d_device->setDCOffsetMode(SOAPY_SDR_RX, channel, dc_offset_auto_mode);
    d_dc_offset_auto_mode = dc_offset_auto_mode;
  }
}

void
source_impl::set_frequency_correction(size_t channel,
                                      double freq_correction)
{
  if (channel >= d_nchan) {
    return;
  }

  if (!hasFrequencyCorrection(channel)) {
    return;
  }

  /* If the device supports Frequency correction set value */
  if (d_device->hasFrequencyCorrection(SOAPY_SDR_RX, channel)) {
    d_device->setFrequencyCorrection(SOAPY_SDR_RX, channel,
                                     freq_correction);
    d_frequency_correction = freq_correction;
  }
}

void
source_impl::set_iq_balance(size_t channel, gr_complexd iq_balance)
{
  if (channel >= d_nchan) {
    return;
  }

  if (!hasIQBalance(channel)) {
    return;
  }

  /* If the device supports IQ blance correction set value */
  if (d_device->hasIQBalance(SOAPY_SDR_RX, channel)) {
    d_device->setIQBalance(SOAPY_SDR_RX, channel, iq_balance);
    d_iq_balance = iq_balance;
  }
}

void
source_impl::set_master_clock_rate(double clock_rate)
{
  d_device->setMasterClockRate(clock_rate);
  d_clock_rate = clock_rate;
}

void
source_impl::set_clock_source(const std::string &clock_source)
{
  d_device->setClockSource(clock_source);
  d_clock_source = clock_source;
}

void
source_impl::set_frontend_mapping(const std::string &mapping)
{
  d_device->setFrontendMapping(SOAPY_SDR_RX, mapping);
}

double
source_impl::get_frequency(size_t channel)
{
  return d_device->getFrequency(SOAPY_SDR_RX, channel);
}

double
source_impl::get_gain(size_t channel)
{
  return d_device->getGain(SOAPY_SDR_RX, channel);
}

bool
source_impl::get_gain_mode(size_t channel)
{
  return d_device->getGainMode(SOAPY_SDR_RX, channel);
}

double
source_impl::get_sampling_rate(size_t channel)
{
  return d_device->getSampleRate(SOAPY_SDR_RX, channel);
}

double
source_impl::get_bandwidth(size_t channel)
{
  return d_device->getBandwidth(SOAPY_SDR_RX, channel);
}

std::string
source_impl::get_antenna(size_t channel)
{
  return d_device->getAntenna(SOAPY_SDR_RX, channel);
}

std::complex<double>
source_impl::get_dc_offset(size_t channel)
{
  return d_device->getDCOffset(SOAPY_SDR_RX, channel);
}

bool
source_impl::get_dc_offset_mode(size_t channel)
{
  return d_device->getDCOffsetMode(SOAPY_SDR_RX, channel);
}

double
source_impl::get_frequency_correction(size_t channel)
{
  return d_device->getFrequencyCorrection(SOAPY_SDR_RX, channel);
}

std::complex<double>
source_impl::get_iq_balance(size_t channel)
{
  return d_device->getIQBalance(SOAPY_SDR_RX, channel);
}

double
source_impl::get_master_clock_rate()
{
  return d_device->getMasterClockRate();
}

std::string
source_impl::get_clock_source()
{
  return d_device->getClockSource();
}

void
source_impl::cmd_handler_frequency(pmt::pmt_t val, size_t chann)
{
  set_frequency(chann, pmt::to_float(val));
}

void
source_impl::cmd_handler_gain(pmt::pmt_t val, size_t chann)
{
  set_gain(chann, pmt::to_float(val));
}

void
source_impl::cmd_handler_samp_rate(pmt::pmt_t val, size_t chann)
{
  set_sample_rate(chann, pmt::to_float(val));
}

void
source_impl::cmd_handler_bw(pmt::pmt_t val, size_t chann)
{
  set_bandwidth(chann, pmt::to_float(val));
}

void
source_impl::cmd_handler_antenna(pmt::pmt_t val, size_t chann)
{
  set_antenna(chann, pmt::symbol_to_string(val));
}

int
source_impl::work(int noutput_items,
                  gr_vector_const_void_star &input_items,
                  gr_vector_void_star &output_items)
{
  long long int time_ns = 0;
  int flags = 0;
  while (1) {
    boost::this_thread::disable_interruption disable_interrupt;
    int read = d_device->readStream(d_stream, output_items.data(), noutput_items,
                                    flags, time_ns);
    boost::this_thread::restore_interruption restore_interrupt(disable_interrupt);
    if (read > 0) {
      return read;
    }

    if (read < 0) {
      // Added some error handling
      switch (read) {
      case SOAPY_SDR_OVERFLOW:
        std::cout << "sO";
        break;
      case SOAPY_SDR_UNDERFLOW:
        std::cout << "sU";
        break;
      case SOAPY_SDR_STREAM_ERROR:
        GR_LOG_WARN(d_logger, boost::format("Block stream error."));
        return 0;;
      case SOAPY_SDR_TIMEOUT:
        break;
      case SOAPY_SDR_CORRUPTION:
        GR_LOG_WARN(d_logger, boost::format("Block corruption."));
        return 0;
      default:
        GR_LOG_WARN(d_logger,
                    boost::format("Block caught RX error code: %d") % read);
        return 0;
      }
    }
  }
  return 0;
}

void
source_impl::msg_handler_command(pmt::pmt_t msg)
{
  if (!pmt::is_dict(msg)) {
    return;
  }
  size_t chann = 0;
  if (pmt::dict_has_key(msg, CMD_CHAN_KEY)) {
    chann = pmt::to_long(
              pmt::dict_ref(msg, CMD_CHAN_KEY, pmt::from_long(0)));
    pmt::dict_delete(msg, CMD_CHAN_KEY);
  }
  for (size_t i = 0; i < pmt::length(msg); i++) {
    d_cmd_handlers[pmt::car(pmt::nth(i, msg))](
      pmt::cdr(pmt::nth(i, msg)), chann);
  }
}
} /* namespace soapy */
} /* namespace gr */

