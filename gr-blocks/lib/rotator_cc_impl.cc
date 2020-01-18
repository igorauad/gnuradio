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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rotator_cc_impl.h"
#include <gnuradio/io_signature.h>

#include <cmath>

namespace gr {
namespace blocks {

rotator_cc::sptr rotator_cc::make(double phase_inc, bool tag_inc_updates)
{
    return gnuradio::get_initial_sptr(new rotator_cc_impl(phase_inc,
                                                          tag_inc_updates));
}

rotator_cc_impl::rotator_cc_impl(double phase_inc, bool tag_inc_updates)
    : sync_block("rotator_cc",
                 io_signature::make(1, 1, sizeof(gr_complex)),
                 io_signature::make(1, 1, sizeof(gr_complex))),
    d_tag_inc_updates(tag_inc_updates),
    d_idx_next_inc_update(0),
    d_next_phase_inc(0.0),
    d_inc_update_pending(false)
{
    set_phase_inc(phase_inc);

    const pmt::pmt_t port_id = pmt::mp("phase_inc");
    message_port_register_in(port_id);
    set_msg_handler(
        port_id,
        [this](pmt::pmt_t msg){ this->handle_phase_inc_msg(msg); });
}

rotator_cc_impl::~rotator_cc_impl() {}

void rotator_cc_impl::set_phase_inc(double phase_inc)
{
    d_r.set_phase_incr(exp(gr_complex(0, phase_inc)));
}

void rotator_cc_impl::handle_phase_inc_msg(pmt::pmt_t msg)
{
    const pmt::pmt_t inc_key    = pmt::intern("inc");
    const pmt::pmt_t offset_key = pmt::intern("offset");

    if (pmt::is_dict(msg)) {
        /* New phase increment (mandatory) */
        if (!pmt::dict_has_key(msg, inc_key))
            throw std::runtime_error("rotator_cc: Control message "
                                     "must contain \"inc\" key");

        const double phase_inc = pmt::to_double(pmt::dict_ref(msg, inc_key,
                                                              pmt::PMT_NIL));

        /* Absolute sample offset when update is to be applied (optional) */
        const uint64_t update_offset = pmt::dict_has_key(msg, offset_key) ?
            pmt::to_uint64(pmt::dict_ref(msg, offset_key, pmt::PMT_NIL)) :
            nitems_written(0) + 1; // update right away when not provided

        d_inc_update_queue.push(std::make_pair(update_offset, phase_inc));
    } else {
        throw std::runtime_error("rotator_cc: Control message must be a PMT "
                                 "dictionary");
    }
}

int rotator_cc_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    const gr_complex* in = (const gr_complex*)input_items[0];
    gr_complex* out = (gr_complex*)output_items[0];

#if 0
      for (int i=0; i<noutput_items; i++)
      	out[i] = d_r.rotate(in[i]);
#else

    const uint64_t n_written = nitems_written(0);

    /* Find out if there is a phase increment update pending */
    while (!d_inc_update_queue.empty() && !d_inc_update_pending) {
        auto next_update = d_inc_update_queue.front();
        d_inc_update_queue.pop();

        if (next_update.first <= n_written)
            continue; // we didn't process this update on time - skip it

        d_idx_next_inc_update = next_update.first;
        d_next_phase_inc      = next_update.second;

        d_inc_update_pending  = true;
    }

    /* If there is a phase increment update scheduled for now, handle rotation
     * in two steps and update the phase increment in between. */
    int items_before_update, items_after_update;
    if (d_inc_update_pending &&
        d_idx_next_inc_update > n_written &&
        d_idx_next_inc_update <= (n_written + noutput_items)) {

        items_before_update = d_idx_next_inc_update - n_written - 1;
        /* NOTE: the rotator operator first rotates the sample, then updates its
         * phase accumulator. This means that the n-th sample is rotated with
         * the phase increment that the rotator had when processing sample
         * n-1. Hence, in order for the update to start taking effect on sample
         * n, we must ensure that on sample n-1 the rotator already has the new
         * increment. The "-1" term is meant to address this. */
        items_after_update  = noutput_items - items_before_update;

        d_r.rotateN(out, in, items_before_update);
        set_phase_inc(d_next_phase_inc);
        d_r.rotateN(out + items_before_update, in + items_before_update,
                    items_after_update);

        if (d_tag_inc_updates) {
            add_item_tag(0,
                         nitems_written(0) + items_before_update + 1,
                         pmt::string_to_symbol("new_inc"),
                         pmt::from_float(d_next_phase_inc));
        }

        d_inc_update_pending = false;
    } else
        d_r.rotateN(out, in, noutput_items);
#endif

    return noutput_items;
}

} /* namespace blocks */
} /* namespace gr */
