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

namespace gr {
  namespace soapy {

    sink::sptr
    sink::make(float frequency, float gain, float samp_rate, float bandwidth, const std::string device)
    {
      return gnuradio::get_initial_sptr
        (new sink_impl(frequency, gain, samp_rate, bandwidth, device));
    }

    /*
     * The private constructor
     */
    sink_impl::sink_impl(float frequency, float gain, float samp_rate, float bandwidth, const std::string device)
      : gr::sync_block("sink",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0))
    {}

    /*
     * Our virtual destructor.
     */
    sink_impl::~sink_impl()
    {
    }

    int
    sink_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
//      const float *in = (const float *) input_items[0];
//      <+OTYPE+> *out = (<+OTYPE+> *) output_items[0];

      // Do <+signal processing+>

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace soapy */
} /* namespace gr */

