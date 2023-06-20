// -*- c++ -*- //
// Copyright (c) Microsoft Corporation.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#ifndef INCLUDED_DIFI_SOURCE_CPP_H
#define INCLUDED_DIFI_SOURCE_CPP_H

#include <difi/api.h>
#include <difi/difi_timestamp_source.h>
#include <gnuradio/sync_block.h>
#include <cstdint>

namespace gr {
  namespace difi {

    template <class T, class S>
    class DIFI_API difi_source_cpp : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<difi_source_cpp<T, S>> sptr;

      static sptr make(std::string ip_addr, uint32_t port, uint8_t socket_type, int stream_number, int context_pkt_behavior, enum difi_timestamp_source timestamp_source);

    };
    typedef difi_source_cpp<gr_complex, std::complex<int16_t>> difi_source_cpp_sc16_fc32;
    typedef difi_source_cpp<gr_complex, std::complex<int8_t>> difi_source_cpp_sc8_fc32;
    typedef difi_source_cpp<std::complex<int8_t>, std::complex<int16_t>> difi_source_cpp_sc16_sc8;
    typedef difi_source_cpp<std::complex<int8_t>, std::complex<int8_t>> difi_source_cpp_sc8_sc8;

  } // namespace difi
} // namespace gr

#endif /* INCLUDED_DIFI_SOURCE_CPP_H */

