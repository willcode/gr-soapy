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


#ifndef INCLUDED_SOAPY_SINK_H
#define INCLUDED_SOAPY_SINK_H

#include <soapy/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace soapy {


    /*!
     * \brief Sink block implements SoapySDR functionality for TX
     * \ingroup soapy
     *
     * The soapy sink block reads a stream and transmits the samples.
     * The sink block also provides Soapy API calls for transmitter settings.
     * Device is a string containing the driver and type name of the
     * device the user wants to use according to the Soapy* module
     * documentation.
     * Make parameters are passed through the xml block.
     * Antenna and clock source can be left empty and default values
     * will be used.
     * This block has a message port, which consumes PMT messages.
     * For a description of the command syntax , see \ref cmd_handler.
     */
    class SOAPY_API sink : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<sink> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of soapy::sink.
       *
       * To avoid accidental use of raw pointers, soapy::sink's
       * constructor is in a private implementation
       * class. soapy::sink::make is the public interface for
       * creating new instances.
       * @param nchan number of channels
       * @param device the device driver and type
       *
       * Driver name can be any of "uhd", "lime", "airspy",
       * "rtlsdr" or others
       */
      static sptr make(size_t nchan, const std::string device);

      /* Callbacks for source fields */
      virtual void set_gain(size_t channel, float gain) = 0;

      virtual void set_frequency(size_t channel, float freq) = 0;

      virtual void set_gain_mode(size_t channel, float gain, bool gain_auto_mode) = 0;

      virtual void set_sample_rate(size_t channel, float sample_rate) = 0;

      virtual void set_bandwidth(size_t channel, float bandwidth) = 0;

      virtual void set_antenna(size_t channel, const std::string &name) = 0;

      virtual void set_dc_offset(size_t channel, gr_complexd dc_offset, bool dc_offset_auto_mode) = 0;

      virtual void set_dc_offset_mode(size_t channel, bool dc_offset_auto_mode) = 0;

      virtual void set_frequency_correction(size_t channel, double freq_correction) = 0;

      virtual void set_iq_balance(size_t channel, gr_complexd iq_balance) = 0;

      virtual void set_master_clock_rate(double clock_rate) = 0;

      virtual void set_clock_source(const std::string &clock_source) = 0;
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SINK_H */

