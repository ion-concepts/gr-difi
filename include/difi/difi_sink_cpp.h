// -*- c++ -*- //
// Copyright (c) Microsoft Corporation and Welkin Sciences, LLC.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#ifndef INCLUDED_DIFI_SINK_CPP_H
#define INCLUDED_DIFI_SINK_CPP_H

#include <difi/api.h>
#include <gnuradio/sync_block.h>
#include <cstdint>

namespace gr {
  namespace difi {
  
    template <class T, class S>
    class DIFI_API difi_sink_cpp : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<difi_sink_cpp<T, S>> sptr;
      
      static sptr make(u_int32_t reference_time_full, u_int64_t reference_time_frac, std::string ip_addr, uint32_t port, uint8_t socket_type, bool mode, uint32_t samples_per_packet, 
                      int stream_number, u_int64_t samp_rate, int context_interval, int context_pack_size, float rf_gain_dB, float if_gain_dB);
    };
    typedef difi_sink_cpp<gr_complex, std::complex<int16_t>> difi_sink_cpp_sc16_fc32;
    typedef difi_sink_cpp<gr_complex, std::complex<int8_t>> difi_sink_cpp_sc8_fc32;
    typedef difi_sink_cpp<std::complex<int8_t>, std::complex<int16_t>> difi_sink_cpp_sc16_sc8;
    typedef difi_sink_cpp<std::complex<int8_t>, std::complex<int8_t>> difi_sink_cpp_sc8_sc8;

  } // namespace difi
} // namespace gr

#endif /* INCLUDED_DIFI_SINK_CPP_H */
