#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) Daniel Estevez.
# Licensed under the GNU General Public License v3.0 or later.
# See License.txt in the project root for license information.
#

from gnuradio import gr, blocks, gr_unittest
try:
    from difi import synchronize
except ImportError:
    import os
    import sys
    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from difi import synchronize
import numpy as np
import pmt


class qa_synchronize(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        instance = synchronize(gr.sizeof_gr_complex, False)
        instance = synchronize(gr.sizeof_float, True)

    def test_output_and_tags(self):
        nsamples = 10000
        data_0 = np.random.randn(nsamples).astype('float32')
        data_1 = np.random.randn(nsamples).astype('float32')
        tag_offsets_0 = [0, 27, 235]
        tag_offsets_1 = [23, 1730, 2415]
        tags_0 = [
            gr.tag_utils.python_to_tag((
                j, pmt.intern('test_tag'),
                pmt.PMT_NIL, pmt.intern('src')))
            for j in tag_offsets_0]
        tags_1 = [
            gr.tag_utils.python_to_tag((
                j, pmt.intern('test_tag'),
                pmt.PMT_NIL, pmt.intern('src')))
            for j in tag_offsets_0]
        source_0 = blocks.vector_source_f(
            data_0, False, 1, tags_0)
        source_1 = blocks.vector_source_f(
            data_0, False, 1, tags_0)
        sync_share_tags = synchronize(gr.sizeof_float, True)
        sync_dont_share_tags = synchronize(gr.sizeof_float, False)
        sink_share_tags = blocks.vector_sink_f(1, nsamples)
        sink_dont_share_tags = blocks.vector_sink_f(1, nsamples)
        self.tb.connect(source_0, sync_share_tags, sink_share_tags)
        self.tb.connect(source_0, sync_dont_share_tags, sink_dont_share_tags)
        self.tb.connect(source_1, (sync_share_tags, 1))
        self.tb.connect(source_1, (sync_dont_share_tags, 1))

        self.tb.run()

        np.testing.assert_equal(sink_share_tags.data(), data_0)
        np.testing.assert_equal(sink_dont_share_tags.data(), data_0)
        self.assertEqual(len(sink_share_tags.tags()),
                         len(tag_offsets_0) + len(tag_offsets_1))
        self.assertEqual(len(sink_dont_share_tags.tags()),
                         len(tag_offsets_0))
        for tag in sink_share_tags.tags():
            self.assertTrue(tag.offset in tag_offsets_0 + tag_offsets_1)
            self.assertEqual(pmt.symbol_to_string(tag.key), 'test_tag')
            self.assertTrue(pmt.is_null(tag.value))
        for tag in sink_share_tags.tags():
            self.assertTrue(tag.offset in tag_offsets_0)
            self.assertEqual(pmt.symbol_to_string(tag.key), 'test_tag')
            self.assertTrue(pmt.is_null(tag.value))


if __name__ == '__main__':
    gr_unittest.run(qa_synchronize)
