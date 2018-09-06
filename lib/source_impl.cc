/* -*- c++ -*- */
/* 
 * Copyright 2018 gr-soapy author.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "source_impl.h"

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
            d_channel(channel)
    {
      makeDevice (device);
      set_frequency (d_channel, d_frequency);
      set_gain (d_channel ,d_gain);
      set_sample_rate (d_channel, d_sampling_rate);
      set_bandwidth (d_channel, d_bandwidth);
      set_antenna (0, d_antenna);
      d_stream = d_device->setupStream (SOAPY_SDR_RX, "CF32");
      d_device->activateStream (d_stream);
      d_mtu = d_device->getStreamMTU (d_stream);
      d_bufs.resize (1);
    }

    source_impl::~source_impl ()
    {
      unmakeDevice (d_device);

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
      std::cout << "Antennas" << std::endl;
      for (auto it : d_device->listAntennas(SOAPY_SDR_TX,0))
        std::cout << it << std::endl;
      d_device->setAntenna (SOAPY_SDR_RX, channel, name);
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

  } /* namespace soapy */
} /* namespace gr */

