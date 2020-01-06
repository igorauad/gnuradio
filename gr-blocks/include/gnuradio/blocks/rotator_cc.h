/* -*- c++ -*- */
/*
 * Copyright 2014 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_BLOCKS_ROTATOR_CC_H
#define INCLUDED_BLOCKS_ROTATOR_CC_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace blocks {

/*!
 * \brief Complex rotator
 * \ingroup math_operators_blk
 *
 * \details
 *
 * Message Ports:
 *
 * phase_inc (input):
 *
 *    Receives a PMT dictionary with a new phase increment to be set on the
 *    rotator at a specified output sample index. The new increment should be
 *    provided with key "inc" and given as a PMT double. The index (as an
 *    absolute output item number) on which the phase increment update should be
 *    handled should be provided with key "offset" and given as a PMT
 *    uint64. The "offset" key is optional and, when not provided, the new phase
 *    increment is configured in the rotator immediately.
 */
class BLOCKS_API rotator_cc : virtual public sync_block
{
public:
    // gr::blocks::rotator_cc::sptr
    typedef boost::shared_ptr<rotator_cc> sptr;

    /*!
     * \brief Make an complex rotator block
     * \param phase_inc rotational velocity
     * \param tag_inc_updates if phase increment is updated following the
     * reception of a control message received via the input message port, tag
     * the sample where the new increment starts to take effect.
     */
    static sptr make(double phase_inc = 0.0, bool tag_inc_updates = false);

    virtual void set_phase_inc(double phase_inc) = 0;
};

} /* namespace blocks */
} /* namespace gr */

#endif /* INCLUDED_BLOCKS_ROTATOR_CC_H */
