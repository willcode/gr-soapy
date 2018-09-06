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


#ifndef INCLUDED_SOAPY_SOURCE_H
#define INCLUDED_SOAPY_SOURCE_H

#include <soapy/api.h>
#include <gnuradio/sync_block.h>




namespace gr {
  namespace soapy {

    /*!
     * \brief <+description of block+>
     * \ingroup soapy
     *
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
       */
      static sptr make(float frequency, float gain, float sampling_rate,
                       float bandwidth, const std::string antenna,
                       size_t channel, const std::string device);

      /* Callbacks for source fields */
      virtual void set_gain(size_t channel, float gain) = 0;

      virtual void set_frequency(size_t channel, float freq) = 0;
    };

  } // namespace soapy
} // namespace gr

#endif /* INCLUDED_SOAPY_SOURCE_H */

