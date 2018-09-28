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
     * \brief <+description of block+>
     * \ingroup soapy
     *
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
       */
      static sptr make(float frequency, float gain, float samp_rate, float bandwidth, const std::string device);
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SINK_H */

