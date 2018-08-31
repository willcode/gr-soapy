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
                  float bandwidth, const std::string device)
    {
      return gnuradio::get_initial_sptr (
          new source_impl (frequency, gain, sampling_rate, bandwidth, device));
    }

    /*
     * The private constructor
     */
    source_impl::source_impl (float frequency, float gain, float sampling_rate,
                              float bandwidth, const std::string device) :
            gr::sync_block ("source", gr::io_signature::make (0, 0, 0),
                            gr::io_signature::make (1, 1, sizeof(gr_complex)))
    {
      makeDevice (device);
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
    source_impl::set_frequency (SoapySDR::Device* dev, float frequency)
    {
      dev->setFrequency (SOAPY_SDR_RX, 0, frequency);
    }

    void
    source_impl::set_gain (SoapySDR::Device* dev, float gain)
    {
      dev->setGain (SOAPY_SDR_RX, 0, gain);
    }

    void
    source_impl::set_sample_rate (SoapySDR::Device* dev, float sample_rate)
    {
      dev->setSampleRate (SOAPY_SDR_RX, 0, sample_rate);
    }

    void
    source_impl::set_bandwidth (SoapySDR::Device* dev, float bandwidth)
    {
      dev->setBandwidth (SOAPY_SDR_RX, 0, bandwidth);
    }

    int
    source_impl::work (int noutput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      // Do <+signal processing+>

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace soapy */
} /* namespace gr */

