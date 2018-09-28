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
#include "source_impl.h"

const pmt::pmt_t CMD_CHAN_KEY = pmt::mp("chan");
const pmt::pmt_t CMD_FREQ_KEY = pmt::mp("freq");
const pmt::pmt_t CMD_GAIN_KEY = pmt::mp("gain");
const pmt::pmt_t CMD_ANTENNA_KEY = pmt::mp("antenna");
const pmt::pmt_t CMD_RATE_KEY = pmt::mp("samp_rate");
const pmt::pmt_t CMD_BW_KEY = pmt::mp("bw");


namespace gr
{
  namespace soapy
  {
    source::sptr
    source::make (float frequency, float gain, float sampling_rate,
                  float bandwidth, const std::string antenna,
                  size_t channel, const std::string device)
    {
      return gnuradio::get_initial_sptr (
          new source_impl (frequency, gain, sampling_rate, bandwidth, antenna,
                           channel, device));
    }

    /*
     * The private constructor
     */
    source_impl::source_impl (float frequency, float gain, float sampling_rate,
                              float bandwidth, const std::string antenna,
                              size_t channel, const std::string device) :
            gr::sync_block ("source", gr::io_signature::make (0, 0, 0),
                            gr::io_signature::make (1, 1, sizeof(gr_complex))),
            d_mtu (0),
            d_frequency (frequency),
            d_gain (gain),
            d_sampling_rate (sampling_rate),
            d_bandwidth (bandwidth),
            d_antenna (antenna),
            d_channel(channel),
            d_message_port(pmt::mp("command"))
    {
      makeDevice (device);
      set_frequency (d_channel, d_frequency);
      set_gain (d_channel ,d_gain);
      set_sample_rate (d_channel, d_sampling_rate);
      set_bandwidth (d_channel, d_bandwidth);
      if(d_antenna.compare("") != 0)
      {
        set_antenna (0, d_antenna);
      }
      d_stream = d_device->setupStream (SOAPY_SDR_RX, "CF32");
      d_device->activateStream (d_stream);
      d_mtu = d_device->getStreamMTU (d_stream);
      d_bufs.resize (1);

      message_port_register_in (d_message_port);
      set_msg_handler (
          d_message_port,
          boost::bind (&source_impl::msg_handler_command, this, _1));

      register_msg_cmd_handler (
          CMD_FREQ_KEY,
          boost::bind (&source_impl::cmd_handler_frequency, this, _1, _2));
      register_msg_cmd_handler (
          CMD_GAIN_KEY,
          boost::bind (&source_impl::cmd_handler_gain, this, _1, _2));
      register_msg_cmd_handler (
          CMD_RATE_KEY,
          boost::bind (&source_impl::cmd_handler_samp_rate, this, _1, _2));
      register_msg_cmd_handler (
          CMD_BW_KEY,
          boost::bind (&source_impl::cmd_handler_bw, this, _1, _2));
      register_msg_cmd_handler (
          CMD_ANTENNA_KEY,
          boost::bind (&source_impl::cmd_handler_antenna, this, _1, _2));
    }

    source_impl::~source_impl ()
    {
      unmakeDevice (d_device);

    }

    void source_impl::register_msg_cmd_handler(const pmt::pmt_t &cmd, cmd_handler_t handler)
    {
      d_cmd_handlers[cmd] = handler;
    }

    int
    source_impl::makeDevice (const std::string &argStr)
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
    source_impl::unmakeDevice (SoapySDR::Device* dev)
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
    source_impl::set_frequency (size_t channel, float frequency)
    {
      d_device->setFrequency (SOAPY_SDR_RX, channel, frequency);
    }

    void
    source_impl::set_gain (size_t channel, float gain)
    {
      d_device->setGain (SOAPY_SDR_RX, channel, gain);
    }

    void
    source_impl::set_sample_rate (size_t channel, float sample_rate)
    {
      d_device->setSampleRate (SOAPY_SDR_RX, channel, sample_rate);
    }

    void
    source_impl::set_bandwidth (size_t channel, float bandwidth)
    {
      d_device->setBandwidth (SOAPY_SDR_RX, channel, bandwidth);
    }

    void
    source_impl::set_antenna (const size_t channel, const std::string &name)
    {
      d_device->setAntenna (SOAPY_SDR_RX, channel, name);
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
    source_impl::work (int noutput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      d_bufs[0] = (gr_complex*) output_items[0];
      int flags = 0;
      long long timeNs = 0;
      size_t total_samples = std::min (noutput_items, (int) d_mtu);
      d_device->readStream (d_stream, &d_bufs[0], total_samples, flags, timeNs,
                            long (1e6));

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void
    source_impl::msg_handler_command (pmt::pmt_t msg)
    {
      if (!pmt::is_dict(msg)) {
        return;
      }
      size_t chann = 0;
      if (pmt::dict_has_key(msg, CMD_CHAN_KEY)) {
        chann = pmt::to_long(
            pmt::dict_ref(
              msg, CMD_CHAN_KEY,
              pmt::from_long(0)
            )
        );
        pmt::dict_delete(msg, CMD_CHAN_KEY);
      }
      for (size_t i = 0; i < pmt::length(msg); i++) {
        d_cmd_handlers[pmt::car(pmt::nth(i, msg))](pmt::cdr(pmt::nth(i, msg)), chann);
      }
    }

  } /* namespace soapy */
} /* namespace gr */

