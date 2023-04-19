/* -*- c++ -*- */
// Copyright (c) Daniel Estevez.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#ifndef INCLUDED_DIFI_SYNCHRONIZE_IMPL_H
#define INCLUDED_DIFI_SYNCHRONIZE_IMPL_H

#include <difi/synchronize.h>

namespace gr {
namespace difi {

class synchronize_impl : public synchronize {
private:
  const size_t d_itemsize;
  const bool d_share_tags;

public:
  synchronize_impl(size_t itemsize, bool share_tags);
  ~synchronize_impl() override;

  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;
};

} // namespace difi
} // namespace gr

#endif /* INCLUDED_DIFI_SYNCHRONIZE_IMPL_H */
