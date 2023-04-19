/* -*- c++ -*- */
// Copyright (c) Daniel Estevez.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#ifndef INCLUDED_DIFI_SYNCHRONIZE_H
#define INCLUDED_DIFI_SYNCHRONIZE_H

#include <difi/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace difi {

/*!
 * \brief Synchronize a sample stream to the rate of another one.
 * \ingroup difi
 *
 * This block has two inputs and one output. Samples are consumed from both
 * inputs at the same rate, and passed from the first input to the output. The
 * intended application of this block is to use the stream of samples from a
 * DIFI Source (as second input) to apply backpressure to a modulator or
 * transmitter connected directly to the the first input of this block and
 * indirectly to a DIFI Sink through the output of this block.
 *
 * Optionally, tags from the second input can be passed to the output.
 */
class DIFI_API synchronize : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<synchronize> sptr;

  /*!
   * \brief Creates a Synchronize block
   *
   * \param itemsize Size of the items in bytes.
   * \param share_tags Pass tags from second input to output.
   */
  static sptr make(size_t itemsize, bool share_tags);
};

} // namespace difi
} // namespace gr

#endif /* INCLUDED_DIFI_SYNCHRONIZE_H */
