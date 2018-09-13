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
#include <boost/bind.hpp>

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/ConverterRegistry.hpp>

typedef boost::function<void(pmt::pmt_t , size_t)> cmd_handler_t;

namespace gr {
  namespace soapy {

    class source_impl : public source
    {
     private:
      SoapySDR::Device* d_device;
      SoapySDR::Stream* d_stream;

      size_t d_mtu;
      std::vector<void*> d_bufs;
      pmt::pmt_t d_message_port;

      float d_frequency;
      float d_gain;
      float d_sampling_rate;
      float d_bandwidth;
      std::string d_antenna;
      size_t d_channel;
      std::map<pmt::pmt_t, cmd_handler_t> d_cmd_handlers;
      gr_complex d_dc_offset;
      bool d_dc_offset_mode;
      double d_frequency_correction;
      gr_complex d_iq_balance;
      bool d_gain_mode;
      double d_clock_rate;
      std::string d_clock_source;
      std::string d_frontend_mapping;

      int makeDevice(const std::string &argStr);
      int unmakeDevice(SoapySDR::Device* dev);
      void set_frequency (size_t channel, float frequency);
      void set_gain(size_t channel, float gain);
      void set_gain_mode(size_t channel, float gain, bool automatic);
      void set_sample_rate(size_t channel, float sample_rate);
      void set_bandwidth(size_t channel, float bandwidth);
      void set_antenna(size_t channel, const std::string &name);
      void set_dc_offset(size_t channel, gr_complexd dc_offset, bool boolean);
      void set_dc_offset_mode(size_t channel, bool automatic);
      void set_frequency_correction(size_t channel, double correction);
      void set_iq_balance(size_t channel, gr_complexd balance);
      void set_master_clock_rate(double rate);
      void set_clock_source(const std::string &name);
      void set_frontend_mapping(const std::string &mapping);
      void msg_handler_command(pmt::pmt_t msg);
      void cmd_handler_frequency(pmt::pmt_t val, size_t chann);
      void cmd_handler_gain(pmt::pmt_t val, size_t chann);
      void cmd_handler_samp_rate(pmt::pmt_t val, size_t chann);
      void cmd_handler_bw(pmt::pmt_t val, size_t chann);
      void cmd_handler_antenna(pmt::pmt_t val, size_t chann);
      void register_msg_cmd_handler(const pmt::pmt_t &cmd, cmd_handler_t handler);



     public:
      source_impl(float frequency, float gain, float sampling_rate,
                  float bandwidth, const std::string antenna, size_t channel,
                  gr_complexd dc_offset, bool dc_offset_mode,
                  double correction, gr_complexd balance,
                  const std::string clock_source, const std::string device);
      ~source_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);

    };
  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SOURCE_IMPL_H */

