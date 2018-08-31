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

#ifndef INCLUDED_SOAPY_SOURCE_IMPL_H
#define INCLUDED_SOAPY_SOURCE_IMPL_H

#include <soapy/source.h>

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/ConverterRegistry.hpp>

namespace gr {
  namespace soapy {

    class source_impl : public source
    {
     private:
      void set_frequency (SoapySDR::Device* dev, float frequency);
      void set_gain(SoapySDR::Device* dev, float gain);
      void set_sample_rate(SoapySDR::Device* dev, float sample_rate);
      void set_bandwidth(SoapySDR::Device* dev, float bandwidth);

     public:
      source_impl(float frequency, float gain, float sampling_rate,
                  float bandwidth, const std::string device);
      ~source_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SOURCE_IMPL_H */

