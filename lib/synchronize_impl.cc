/* -*- c++ -*- */
// Copyright (c) Daniel Estevez.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#include "synchronize_impl.h"
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <cstring>
#include <vector>

namespace gr {
namespace difi {

synchronize::sptr synchronize::make(size_t itemsize, bool share_tags) {
  return gnuradio::make_block_sptr<synchronize_impl>(itemsize, share_tags);
}

synchronize_impl::synchronize_impl(size_t itemsize, bool share_tags)
    : gr::sync_block(
          "synchronize",
          gr::io_signature::make(2, 2,
                                 itemsize),
          gr::io_signature::make(1, 1,
                                 itemsize)),
      d_itemsize(itemsize),
      d_share_tags(share_tags) {
  if (share_tags) {
    set_tag_propagation_policy(TPP_ALL_TO_ALL);
  } else {
    // Tag propagation done manually in work(), since TPP_ONE_TO_ONE does not
    // support this use case.
    set_tag_propagation_policy(TPP_DONT);
  }
}

synchronize_impl::~synchronize_impl() {}

int synchronize_impl::work(int noutput_items,
                           gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items) {
  auto in = static_cast<const uint8_t *>(input_items[0]);
  auto out = static_cast<uint8_t *>(output_items[0]);

  std::memcpy(out, in, noutput_items * d_itemsize);

  if (!d_share_tags) {
    std::vector<gr::tag_t> tags;
    get_tags_in_range(tags, 0, nitems_written(0), nitems_written(0) + noutput_items);
    for (const auto& tag : tags) {
      add_item_tag(0, tag.offset, tag.key, tag.value);
    }
  }

  return noutput_items;
}

} /* namespace difi */
} /* namespace gr */
