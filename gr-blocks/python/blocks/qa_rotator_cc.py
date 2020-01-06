#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2019 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import numpy as np
import pmt
import time


class qa_rotator_cc(gr_unittest.TestCase):

    def setUp(self):
        self.n_samples  = 100
        self.f_in       = 0.01  # frequency on rotator's input
        self.f_shift    = 0.015 # rotator's starting frequency
        tag_inc_updates = True

        self.tb = gr.top_block()

        # Input IQ samples
        in_angles  = 2 * np.pi * np.arange(self.n_samples) * self.f_in
        in_samples = np.exp(1j*in_angles)

        # Rotator's starting phase increment
        phase_inc = 2 * np.pi * self.f_shift

        self.source     = blocks.vector_source_c(in_samples)
        self.rotator_cc = blocks.rotator_cc(phase_inc, tag_inc_updates)
        self.sink       = blocks.vector_sink_c()

    def tearDown(self):
        self.tb = None

    def test_001_t(self):
        """Complex sinusoid frequency shift
        """

        f_out = self.f_in + self.f_shift # frequency of rotator's output

        # Expected IQ samples
        expected_angles  = 2 * np.pi * np.arange(self.n_samples) * f_out
        expected_samples = np.exp(1j*expected_angles)

        self.tb.connect(self.source, self.rotator_cc, self.sink)
        self.tb.run()

        for (x,y) in zip(self.sink.data(), expected_samples):
            self.assertComplexAlmostEqual(x, y, places=5)

    def test_002_t(self):
        """Rotator phase increment update via control message
        """

        new_f_shift   = 0.016  # new rotator's frequency
        offset        = 50     # when to apply new increment
        msg_period_ms = 0      # periodicity (in ms) of increment update message

        new_phase_inc = float(2 * np.pi * new_f_shift)
        f_out_1       = self.f_in + self.f_shift # before update
        f_out_2       = self.f_in + new_f_shift  # after update

        # Message to be sent to the rotator in order to update its increment
        ctrl_msg = pmt.cons(pmt.from_uint64(offset),
                            pmt.from_double(new_phase_inc))

        rot_ctrl = blocks.message_strobe(ctrl_msg, msg_period_ms)

        # Rotator will place a tag on the sample where the new increment starts
        tag_sink = blocks.tag_debug(gr.sizeof_gr_complex, "new_inc", "new_inc")

        # Samples before and after increment update
        n_before = offset
        n_after  = self.n_samples - offset

        # Expected IQ samples
        angles_before_update = 2 * np.pi * np.arange(n_before) * f_out_1
        angles_after_update  = angles_before_update[-1] + \
                               2 * np.pi * np.arange(1, n_after+1) * f_out_2
        expected_angles      = np.concatenate((angles_before_update,
                                               angles_after_update))
        expected_samples     = np.exp(1j*expected_angles)

        self.tb.connect(self.source, self.rotator_cc)
        self.tb.msg_connect(rot_ctrl, "strobe", self.rotator_cc, "phase_inc")
        self.tb.connect(self.rotator_cc, self.sink)
        self.tb.connect(self.rotator_cc, tag_sink)

        self.tb.start()
        time.sleep(1)
        self.tb.stop()
        self.tb.wait()

        tags = tag_sink.current_tags()
        self.assertEqual(len(tags), 1)
        for tag in tags:
            self.assertAlmostEqual(pmt.to_double(tag.value), new_phase_inc)

        for i,(x,y) in enumerate(zip(self.sink.data(), expected_samples)):
            try:
                self.assertComplexAlmostEqual(x, y, places=5)
            except AssertionError as e:
                print("Error on sample %d - angle %f != angle %f" %(
                    i, np.angle(x), np.angle(y)))
                raise e


if __name__ == '__main__':
    gr_unittest.run(qa_rotator_cc)
