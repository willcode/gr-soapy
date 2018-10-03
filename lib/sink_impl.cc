/* -*- c++ -*- */
/*
 * gr-soapy: Soapy SDR Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2018
 *  Libre Space Foundation <http://librespacefoundation.org/>
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
#include "sink_impl.h"

const pmt::pmt_t CMD_CHAN_KEY = pmt::mp ("chan");
const pmt::pmt_t CMD_FREQ_KEY = pmt::mp ("freq");
const pmt::pmt_t CMD_GAIN_KEY = pmt::mp ("gain");
const pmt::pmt_t CMD_ANTENNA_KEY = pmt::mp ("antenna");
const pmt::pmt_t CMD_RATE_KEY = pmt::mp ("samp_rate");
const pmt::pmt_t CMD_BW_KEY = pmt::mp ("bw");

namespace gr
{
  namespace soapy
  {

    sink::sptr
    sink::make (float frequency, float gain, float sampling_rate,
                float bandwidth, const std::string antenna, size_t channel,
                gr_complexd dc_offset, bool dc_offset_auto_mode,
                bool gain_auto_mode, double freq_correction,
                gr_complexd iq_balance, const std::string clock_source,
                const std::string device)
    {
      return gnuradio::get_initial_sptr (
          new sink_impl (frequency, gain, sampling_rate, bandwidth, antenna,
                         channel, dc_offset, dc_offset_auto_mode,
                         gain_auto_mode, freq_correction, iq_balance,
                         clock_source, device));
    }

    /*
     * The private constructor
     */
    sink_impl::sink_impl (float frequency, float gain, float sampling_rate,
                          float bandwidth, const std::string antenna,
                          size_t channel, gr_complexd dc_offset,
                          bool dc_offset_auto_mode, bool gain_auto_mode,
                          double freq_correction, gr_complexd iq_balance,
                          const std::string clock_source,
                          const std::string device) :
            gr::sync_block ("sink",
                            gr::io_signature::make (1, 1, sizeof(gr_complex)),
                            gr::io_signature::make (0, 0, 0)),
            d_mtu (0),
            d_message_port (pmt::mp ("command")),
            d_frequency (frequency),
            d_gain (gain),
            d_sampling_rate (sampling_rate),
            d_bandwidth (bandwidth),
            d_antenna (antenna),
            d_channel (channel),
            d_dc_offset (dc_offset),
            d_dc_offset_auto_mode (dc_offset_auto_mode),
            d_gain_auto_mode (gain_auto_mode),
            d_frequency_correction (freq_correction),
            d_iq_balance (iq_balance),
            d_clock_source (clock_source)
    {
      makeDevice (device);
      set_frequency (d_channel, d_frequency);
      set_gain_mode (d_channel, d_gain, d_gain_mode);
      set_sample_rate (d_channel, d_sampling_rate);
      set_bandwidth (d_channel, d_bandwidth);
      if (d_antenna.compare ("") != 0) {
        set_antenna (0, d_antenna);
      }
      set_dc_offset (d_channel, d_dc_offset, d_dc_offset_auto_mode);
      set_dc_offset_mode (d_channel, d_dc_offset_auto_mode);
      set_frequency_correction (d_channel, d_frequency_correction);
      set_iq_balance (d_channel, d_iq_balance);
      if (d_clock_source.compare ("") != 0) {
        set_clock_source (d_clock_source);
      }
      d_stream = d_device->setupStream (SOAPY_SDR_TX, "CF32");
      d_device->activateStream (d_stream);
      d_mtu = d_device->getStreamMTU (d_stream);
      d_bufs.resize (1);

      message_port_register_in (d_message_port);
      set_msg_handler (d_message_port,
                       boost::bind (&sink_impl::msg_handler_command, this, _1));

      register_msg_cmd_handler (
          CMD_FREQ_KEY,
          boost::bind (&sink_impl::cmd_handler_frequency, this, _1, _2));
      register_msg_cmd_handler (
          CMD_GAIN_KEY,
          boost::bind (&sink_impl::cmd_handler_gain, this, _1, _2));
      register_msg_cmd_handler (
          CMD_RATE_KEY,
          boost::bind (&sink_impl::cmd_handler_samp_rate, this, _1, _2));
      register_msg_cmd_handler (
          CMD_BW_KEY, boost::bind (&sink_impl::cmd_handler_bw, this, _1, _2));
      register_msg_cmd_handler (
          CMD_ANTENNA_KEY,
          boost::bind (&sink_impl::cmd_handler_antenna, this, _1, _2));
    }

    /*
     * Our virtual destructor.
     */
    sink_impl::~sink_impl ()
    {
      unmakeDevice (d_device);
    }

    void
    sink_impl::register_msg_cmd_handler (const pmt::pmt_t &cmd,
                                         cmd_handler_t handler)
    {
      d_cmd_handlers[cmd] = handler;
    }

    int
    sink_impl::makeDevice (const std::string &argStr)
    {
      try {
        d_device = SoapySDR::Device::make (argStr);
      }
      catch (const std::exception &ex) {
        std::cerr << "Error making device: " << ex.what () << std::endl;
        return EXIT_FAILURE;
      }
      return EXIT_SUCCESS;
    }

    int
    sink_impl::unmakeDevice (SoapySDR::Device* dev)
    {
      try {
        SoapySDR::Device::unmake (dev);
      }
      catch (const std::exception &ex) {
        std::cerr << "Error unmaking device: " << ex.what () << std::endl;
        return EXIT_FAILURE;
      }
      return EXIT_SUCCESS;
    }

    void
    sink_impl::set_frequency (size_t channel, float frequency)
    {
      d_device->setFrequency (SOAPY_SDR_TX, channel, frequency);
    }

    void
    sink_impl::set_gain (size_t channel, float gain)
    {
      d_device->setGain (SOAPY_SDR_TX, channel, gain);
    }

    void
    sink_impl::set_gain_mode (size_t channel, float gain,
                              bool gain_auto_mode)
    {
      /* If user specifies no automatic gain setting set gain value */
      if (!gain_auto_mode) {
        d_device->setGain (SOAPY_SDR_TX, channel, gain);
      }
      d_device->setGainMode (SOAPY_SDR_TX, channel, gain_auto_mode);
    }

    void
    sink_impl::set_sample_rate (size_t channel, float sample_rate)
    {
      d_device->setSampleRate (SOAPY_SDR_TX, channel, sample_rate);
    }

    void
    sink_impl::set_bandwidth (size_t channel, float bandwidth)
    {
      d_device->setBandwidth (SOAPY_SDR_TX, channel, bandwidth);
    }

    void
    sink_impl::set_antenna (const size_t channel, const std::string &name)
    {
      d_device->setAntenna (SOAPY_SDR_TX, channel, name);
    }

    void
    sink_impl::set_dc_offset (size_t channel, gr_complexd dc_offset,
                              bool dc_offset_auto_mode)
    {
      /* If DC Correction is supported but automatic mode is not set DC correction */
      if (!dc_offset_auto_mode
          && d_device->hasDCOffset (SOAPY_SDR_TX, channel)) {
        d_device->setDCOffset (SOAPY_SDR_TX, channel, dc_offset);
        d_dc_offset = dc_offset;
      }
    }

    void
    sink_impl::set_dc_offset_mode (size_t channel, bool dc_offset_auto_mode)
    {
      /* If user specifies automatic DC Correction and is supported activate it */
      if (dc_offset_auto_mode
          && d_device->hasDCOffsetMode (SOAPY_SDR_TX, channel)) {
        d_device->setDCOffsetMode (SOAPY_SDR_TX, channel, dc_offset_auto_mode);
        d_dc_offset_auto_mode = dc_offset_auto_mode;
      }
    }

    void
    sink_impl::set_frequency_correction (size_t channel, double freq_correction)
    {
      /* If the device supports Frequency correction set value */
      if (d_device->hasFrequencyCorrection (SOAPY_SDR_TX, channel)) {
        d_device->setFrequencyCorrection (SOAPY_SDR_TX, channel,
                                          freq_correction);
        d_frequency_correction = freq_correction;
      }
    }

    void
    sink_impl::set_iq_balance (size_t channel, gr_complexd iq_balance)
    {
      /* If the device supports IQ blance correction set value */
      if (d_device->hasIQBalance (SOAPY_SDR_TX, channel)) {
        d_device->setIQBalance (SOAPY_SDR_TX, channel, iq_balance);
        d_iq_balance = iq_balance;
      }
    }

    void
    sink_impl::set_master_clock_rate (double clock_rate)
    {
      d_device->setMasterClockRate (clock_rate);
      d_clock_rate = clock_rate;
    }

    void
    sink_impl::set_clock_source (const std::string &clock_source)
    {
      d_device->setClockSource (clock_source);
      d_clock_source = clock_source;
    }

    void
    sink_impl::set_frontend_mapping (const std::string &mapping)
    {
      d_device->setFrontendMapping (SOAPY_SDR_TX, mapping);
    }

    double
    sink_impl::get_frequency (size_t channel)
    {
      return d_device->getFrequency (SOAPY_SDR_TX, channel);
    }

    double
    sink_impl::get_gain (size_t channel)
    {
      return d_device->getGain (SOAPY_SDR_TX, channel);
    }

    bool
    sink_impl::get_gain_mode (size_t channel)
    {
      return d_device->getGainMode (SOAPY_SDR_TX, channel);
    }

    double
    sink_impl::get_sampling_rate (size_t channel)
    {
      return d_device->getSampleRate (SOAPY_SDR_TX, channel);
    }

    double
    sink_impl::get_bandwidth (size_t channel)
    {
      return d_device->getBandwidth (SOAPY_SDR_TX, channel);
    }

    std::string
    sink_impl::get_antenna (size_t channel)
    {
      return d_device->getAntenna (SOAPY_SDR_TX, channel);
    }

    std::complex<double>
    sink_impl::get_dc_offset (size_t channel)
    {
      return d_device->getDCOffset (SOAPY_SDR_TX, channel);
    }

    bool
    sink_impl::get_dc_offset_mode (size_t channel)
    {
      return d_device->getDCOffsetMode (SOAPY_SDR_TX, channel);
    }

    double
    sink_impl::get_frequency_correction (size_t channel)
    {
      return d_device->getFrequencyCorrection (SOAPY_SDR_TX, channel);
    }

    std::complex<double>
    sink_impl::get_iq_balance (size_t channel)
    {
      return d_device->getIQBalance (SOAPY_SDR_TX, channel);
    }

    double
    sink_impl::get_master_clock_rate ()
    {
      return d_device->getMasterClockRate ();
    }

    std::string
    sink_impl::get_clock_source ()
    {
      return d_device->getClockSource ();
    }

    void
    sink_impl::cmd_handler_frequency (pmt::pmt_t val, size_t chann)
    {
      set_frequency (chann, pmt::to_float (val));
    }

    void
    sink_impl::cmd_handler_gain (pmt::pmt_t val, size_t chann)
    {
      set_gain (chann, pmt::to_float (val));
    }

    void
    sink_impl::cmd_handler_samp_rate (pmt::pmt_t val, size_t chann)
    {
      set_sample_rate (chann, pmt::to_float (val));
    }

    void
    sink_impl::cmd_handler_bw (pmt::pmt_t val, size_t chann)
    {
      set_bandwidth (chann, pmt::to_float (val));
    }

    void
    sink_impl::cmd_handler_antenna (pmt::pmt_t val, size_t chann)
    {
      set_antenna (chann, pmt::symbol_to_string (val));
    }

    int
    sink_impl::work (int noutput_items, gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
    {
      int ninput_items = noutput_items;
      d_bufs[0] = (const gr_complex*) input_items[0];
      int flags = 0;
      long long timeNs = 0;
      size_t total_samples = ninput_items;
      int write;
      /* While total_samples > MTU write MTU samples to device */
      while (total_samples > d_mtu) {
        write = d_device->writeStream (d_stream, &d_bufs[0], d_mtu, flags,
                                       timeNs, long (1e6));
        total_samples -= write;
        d_bufs[0] += write * sizeof(gr_complex);
      }
      /* If total_samples < MTU write total_samples to device */
      write = d_device->writeStream (d_stream, &d_bufs[0], total_samples, flags,
                                     timeNs, long (1e6));
      total_samples -= write;
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void
    sink_impl::msg_handler_command (pmt::pmt_t msg)
    {
      if (!pmt::is_dict (msg)) {
        return;
      }
      size_t chann = 0;
      if (pmt::dict_has_key (msg, CMD_CHAN_KEY)) {
        chann = pmt::to_long (
            pmt::dict_ref (msg, CMD_CHAN_KEY, pmt::from_long (0)));
        pmt::dict_delete (msg, CMD_CHAN_KEY);
      }
      for (size_t i = 0; i < pmt::length (msg); i++) {
        d_cmd_handlers[pmt::car (pmt::nth (i, msg))] (
            pmt::cdr (pmt::nth (i, msg)), chann);
      }
    }
  } /* namespace soapy */
} /* namespace gr */

