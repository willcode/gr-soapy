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


#ifndef INCLUDED_SOAPY_SOURCE_H
#define INCLUDED_SOAPY_SOURCE_H

#include <soapy/api.h>
#include <gnuradio/sync_block.h>




namespace gr {
  /// \defgroup messages

  /*!
   * \ingroup messages
   * \section cmd_handler PMT message syntax
   * Message handlers expect a dictionary for
   * setting device parameters from asynchronous input messages.
   * Key must be a pmt dictionary mapping the parameter to its
   * value to be updated. pmt::dict keys can be any of the following: \n
   * "chann"     : Corresponding channel index (int)\n
   * "freq"      : Center frequency (Hz) \n
   * "gain"      : Gain value (dB) \n
   * "antenna"   : Antenna (string)\n
   * "samp_rate" : Sampling rate (samples/second)\n
   * "bw"        : Bandwidth (Hz)
   */
  namespace soapy {

    /*!
     * \brief Source block implements SoapySDR functionality for RX.
     * \ingroup soapy
     *
     * The soapy source block receives samples and writes to a stream.
     * The source block also provides Soapy API calls for receiver settings.
     * Includes all parameters for full RX implementation.
     * Device is a string containing the driver and type name of the
     * device the user wants to use according to the Soapy* module
     * documentation.
     * Make parameters are passed through the xml block.
     * Antenna and clock source can be left empty and default values
     * will be used.
     * This block has a message port, which consumes PMT messages.
     * For a description of the command syntax , see \ref cmd_handler.
     */
    class SOAPY_API source : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<source> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of soapy::source.
       *
       * To avoid accidental use of raw pointers, soapy::source's
       * constructor is in a private implementation
       * class. soapy::source::make is the public interface for
       * creating new instances.
       * @param frequency center frequency in Hz
       * @param gain amplification value in dB
       * @param sampling_rate samples per second
       * @param bandwidth the baseband filter width in Hz
       * @param antenna name of an available antenna
       * @param channel an available channel on the device
       * @param dc_offset the relative correction (1.0 max)
       * @param dc_offset_mode true for automatic correction
       * @param frequency_correction the correction value in PPM
       * @param iq_balance the relative correction (1.0 max)
       * @param clock_source the name of clock source
       * @param clock_rate the master clock rate
       * @param device the device driver and type
       *
       * Driver name can be any of "uhd", "lime", "airspy",
       * "rtlsdr" or others
       */
      static sptr make(float frequency, float gain, float sampling_rate,
                       float bandwidth, const std::string antenna,
                       size_t channel, gr_complexd dc_offset,
                       bool dc_offset_mode, double frequency_correction,
                       gr_complexd iq_balance, const std::string clock_source,
                       double clock_rate,
                       const std::string device);

      /* Callbacks for source fields */
      virtual void set_gain(size_t channel, float gain) = 0;

      virtual void set_frequency(size_t channel, float freq) = 0;
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SOURCE_H */

