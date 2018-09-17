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

#ifndef INCLUDED_SOAPY_SINK_IMPL_H
#define INCLUDED_SOAPY_SINK_IMPL_H

#include <soapy/sink.h>

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/ConverterRegistry.hpp>

namespace gr {
  namespace soapy {

    class sink_impl : public sink
    {
     private:
      SoapySDR::Device* d_device;
      SoapySDR::Stream* d_stream;

      size_t d_mtu;
      std::vector<const void*> d_bufs;

      float d_frequency;
      float d_gain;
      float d_sampling_rate;
      float d_bandwidth;
      std::string d_antenna;
      size_t d_channel;

      int makeDevice(const std::string &argStr);
      int unmakeDevice(SoapySDR::Device* dev);
      void set_frequency (size_t channel, float frequency);
      void set_gain(size_t channel, float gain);
      void set_gain_mode(size_t channel, float gain, bool automatic);
      void set_sample_rate(size_t channel, float sample_rate);
      void set_bandwidth(size_t channel, float bandwidth);
      void set_antenna(size_t channel, const std::string &name);

     public:
      sink_impl(float frequency, float gain, float samp_rate, float bandwidth,
                const std::string antenna, size_t channel, const std::string device);
      ~sink_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SINK_IMPL_H */

