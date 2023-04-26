// -*- c++ -*-
// Copyright (c) Microsoft Corporation and Welkin Sciences, LLC.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.

#include <gnuradio/io_signature.h>
#include "difi_sink_cpp_impl.h"
#include <volk/volk.h>

#include "tcp_client.h"
#include "udp_socket.h"

namespace gr {
  namespace difi {

    template<>
    void difi_sink_cpp_impl<gr_complex, std::complex<int8_t>>::pack_samples(
                                                                            std::complex<int8_t>* output_vector,
                                                                            const gr_complex* input_vector,
                                                                            size_t num_samples)
    {
      volk_32f_s32f_convert_8i(reinterpret_cast<int8_t*>(output_vector),
                               reinterpret_cast<const float*>(input_vector),
                               127.0,
                               2 * num_samples);
    }

    template<>
    void difi_sink_cpp_impl<gr_complex, std::complex<int16_t>>::pack_samples(
                                                                             std::complex<int16_t> * output_vector,
                                                                             const gr_complex* input_vector,
                                                                             size_t num_samples)
    {
      volk_32f_s32f_convert_16i(reinterpret_cast<int16_t*>(output_vector),
                                reinterpret_cast<const float*>(input_vector),
                                32767.0,
                                2 * num_samples);
    }

    template<>
    void difi_sink_cpp_impl<std::complex<int8_t>, std::complex<int8_t>>::pack_samples(
                                                                                      std::complex<int8_t>* output_vector,
                                                                                      const std::complex<int8_t>* input_vector,
                                                                                      size_t num_samples)
    {
      
      std::memcpy(output_vector, input_vector, 2 * num_samples);
    }

     template<>
     void difi_sink_cpp_impl<std::complex<int8_t>, std::complex<int16_t>>::pack_samples(
                                                                                        std::complex<int16_t>* output_vector,
                                                                                        const std::complex<int8_t>* input_vector,
                                                                                          size_t num_samples)
     {
      volk_8i_convert_16i(reinterpret_cast<int16_t*>(output_vector),
                          reinterpret_cast<const int8_t*>(input_vector),
                          2 * num_samples);
    }

    template <class T, class S>
    typename difi_sink_cpp<T, S>::sptr
    difi_sink_cpp<T, S>::make(u_int32_t reference_time_full, u_int64_t reference_time_frac, std::string ip_addr, uint32_t port, uint8_t socket_type,
                          bool mode, uint32_t samples_per_packet, int stream_number, u_int64_t samp_rate,
                          int context_interval, int context_pack_size, float rf_gain_dB, float if_gain_dB)

    {
      return gnuradio::make_block_sptr<difi_sink_cpp_impl<T, S>>(reference_time_full, reference_time_frac, ip_addr, port, socket_type, mode,
                                                              samples_per_packet, stream_number, samp_rate, context_interval, context_pack_size, rf_gain_dB, if_gain_dB);
    }

    template <class T, class S>
    difi_sink_cpp_impl<T, S>::difi_sink_cpp_impl(u_int32_t reference_time_full, u_int64_t reference_time_frac, std::string ip_addr,
                                              uint32_t port, uint8_t socket_type, bool mode, uint32_t samples_per_packet, int stream_number,
                                              u_int64_t samp_rate, int context_interval, int context_pack_size, float rf_gain_dB, float if_gain_dB)
      : gr::sync_block("difi_sink_cpp_impl",
              gr::io_signature::make(1, 1, sizeof(T)),
              gr::io_signature::make(0, 0, 0)),
              d_stream_number(int(stream_number)),
              d_full_samp(samp_rate),
              d_pkt_n(0),
              d_current_buff_idx(0),
              d_pcks_since_last_reference(0),
              d_is_paired_mode(mode),
              d_packet_count(0),
              d_context_packet_count(0),
              d_context_packet_size(context_pack_size),
              d_contex_packet_interval(context_interval),
              p_tcpsocket(0),
              p_udpsocket(0)

    {
      socket_type = (socket_type == 1) ?  SOCK_STREAM : SOCK_DGRAM;

      if(socket_type == SOCK_DGRAM)
      {
        p_udpsocket = new udp_socket(ip_addr,port,false);
      }
      else
      {
        p_tcpsocket = new tcp_client(ip_addr,port);
      }

      if (samples_per_packet < 2)
      {
        GR_LOG_ERROR(this->d_logger, "samples per packet cannot be less than 2, (to ensure one 32 bit word is filled)");
        throw std::runtime_error("samples per packet cannot be less than 2, (to ensure one 32 bit word is filled)");
      }
      d_context_key = pmt::intern("context");
      d_pkt_n_key = pmt::intern("pck_n");
      d_static_change_key = pmt::intern("static_change");
      d_full = reference_time_full;
      d_frac = reference_time_frac;
      d_static_bits = 0x18e00000; // header bits 31-20 must be 0x18e (posix), 0x18a (gps), or 0x186 (utc)
      d_context_static_bits = 0x49e00000;// header bits 31-20 must be 0x49e (posix), 0x49a (gps), or 0x496 (utc)
      d_samples_per_packet = samples_per_packet;
      d_time_adj = (double)d_samples_per_packet / samp_rate;
      d_data_len = samples_per_packet * sizeof(S);
      u_int32_t tmp_header_data = d_static_bits ^ d_pkt_n << 16 ^ (d_data_len + difi::DIFI_HEADER_SIZE) / 4;
      u_int32_t tmp_header_context = d_context_static_bits ^ d_context_packet_count << 16 ^ (context_pack_size  / 4);
      u_int64_t d_class_id = d_oui << 32;
      //u_int64_t d_context_class_id = d_class_id ^ 1;
      u_int64_t d_context_class_id = d_class_id ^ 0;
      d_raw.resize(difi::DIFI_HEADER_SIZE);
      pack_u32(&d_raw[0], tmp_header_data);
      pack_u32(&d_raw[4], d_stream_number);
      int idx = 0;
      pack_u64(&d_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], d_class_id);
      pack_u32(&d_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0);
      pack_u64(&d_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0);

      //pack context for standalone mode only
      d_context_raw.resize(context_pack_size);
      pack_u32(&d_context_raw[0], tmp_header_context);
      pack_u32(&d_context_raw[4], d_stream_number);
      idx = 0;
      pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], d_context_class_id);
      // this is static since only 8 or 16 bit signed complex cartesian is supported for now and item packing is always link efficient
      // see 2.2.2 Standard Flow Signal Context Packet in the DIFI spec for complete information
      // (Legacy VITA49.0  Kratos SNNB format shows packing size = data item size.)
      u_int64_t data_payload_format = sizeof(S) == 2 ?
        ( (context_pack_size == 84) ? difi::EIGHT_BIT_SIGNED_CART_LINK_EFF_SNNB : difi::EIGHT_BIT_SIGNED_CART_LINK_EFF)
        : difi::SIXTEEN_BIT_SIGNED_CART_LINK_EFF;
   
    u_int32_t state_and_event_id =difi::DEFAULT_STATE_AND_EVENTS; // default no events or state values. See 7.1.5.17 The State and Event Indicator Field of the VITA spec
    u_int64_t to_vita_bw = u_int64_t(samp_rate * .8) << 20; // no fractional bw or samp rate supported in gnuradio, see 2.2.2 Standard Flow Signal Context Packet for bandwidth information
    u_int64_t to_vita_if_band_offset= int64_t(samp_rate * -0.1) << 20; //generic IF band offset (1Hz resolution)
    u_int64_t to_vita_samp_rate = samp_rate << 20;
    u_int64_t to_vita_if_ref_freq = 100000000UL << 20; //generic 100MHz IF reference frequency
    u_int64_t to_vita_rf_ref_freq = 7500000000UL << 20; //generic 7.5GHz RF reference frequency
    u_int64_t to_vita_ref_level = 20 << 7; //generic 20dBm reference level
    //double rf_gain_dB=14.2, if_gain_dB=-1.3; //generic two-stage (RF then IF) gains/attenuations
    int32_t to_vita_rf_gain = (rf_gain_dB * (1<<7));
    int32_t to_vita_if_gain = (if_gain_dB * (1<<7));
    int32_t to_vita_gain = (to_vita_if_gain << 16) | to_vita_rf_gain;
    double delay_sec = 1e-5; //generic 10 microsecond timestamp adjustment
    int64_t to_vita_delay = delay_sec * 1e15; //convert to femtoseconds

    // Override DIFI values with legacy VITA49.0 Kratos SNNB values
    if (context_pack_size == 84)
      {
        state_and_event_id = difi::DEFAULT_STATE_AND_EVENTS_KRATOS_SNNB; // default no events or state values. See 7.1.5.17 The State and Event Indicator Field of the VITA spec
        to_vita_bw = 0x000010f447100000UL; // no fractional bw or samp rate supported in gnuradio, see 2.2.2 Standard Flow Signal Context Packet for bandwidth information
        to_vita_if_band_offset= int64_t(samp_rate * -0.1) << 20; //generic IF band offset (1Hz resolution)
        to_vita_samp_rate = 0x1312D0000000UL;
        to_vita_if_ref_freq = 0UL; 
        to_vita_rf_ref_freq = 0x047868C0000000UL; 
        to_vita_ref_level = 0xE380U;
        //to_vita_gain = 0x00001A00U; // 52dB "kratos end-to-end-path" gain (-5dBm CW with full swing input at 8b)
      }

    if(context_pack_size == 72)// this check is a temporary work around for a non-compliant hardware device
      {
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], 966885376U);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], to_vita_bw);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], 0);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], 0);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], 0);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], to_vita_samp_rate);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], state_and_event_id);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]], data_payload_format);

      }
    else if (context_pack_size == 84)// this check is a work around for a non-compliant Kratos SNNB @ 1.7.5
      {
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], 0);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], 0);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], 0x39A18000u);
        //pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], 0x64u);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_bw);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_if_ref_freq);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_rf_ref_freq);
        //pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_if_band_offset);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_ref_level);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_gain);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_samp_rate);
        //pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], to_vita_delay);
        //pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], d_full);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], state_and_event_id);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]], data_payload_format);

      }     
      else
      {
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0xfbb98000u);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], 0x64u);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_bw);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_if_ref_freq);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_rf_ref_freq);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_if_band_offset);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_ref_level);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_gain);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_samp_rate);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], to_vita_delay);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], d_full);
        pack_u32(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], state_and_event_id);
        pack_u64(&d_context_raw[difi::CONTEXT_PACKET_OFFSETS[idx++]], data_payload_format);
      }
      d_out_buf.resize(d_data_len);
      d_to_send.resize(difi::DIFI_HEADER_SIZE + d_out_buf.size());
    }

    template <class T, class S>
    difi_sink_cpp_impl<T, S>::~difi_sink_cpp_impl()
    {
      if(p_udpsocket)
        delete p_udpsocket;

      if(p_tcpsocket)
        delete p_tcpsocket;
    }

    template <class T, class S>
    int difi_sink_cpp_impl<T, S>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {

      if(p_tcpsocket)
      {
        if(!p_tcpsocket->is_connected())
        {
          bool res = p_tcpsocket->connect();
          if(!res)
          {
            return 0;
          }
        }
      }

      const T *in = reinterpret_cast<const T*>(input_items[0]);

      if(d_is_paired_mode)
      {
        process_tags(noutput_items);
      }

      int consumed = 0;
      while (consumed < noutput_items)
      {
        const int to_consume = std::min(static_cast<size_t>(noutput_items - consumed),
                                        (d_out_buf.size() - d_current_buff_idx) / sizeof(S));
        pack_samples(reinterpret_cast<S*>(&d_out_buf[d_current_buff_idx]),
                     &in[consumed],
                     to_consume);
        consumed += to_consume;
        d_current_buff_idx += to_consume * sizeof(S);

        if(d_current_buff_idx >= d_out_buf.size())
        {
          if(!d_is_paired_mode and d_packet_count % d_contex_packet_interval == 0)
              send_context();

          pack_data();
          if(p_udpsocket)
          {
            p_udpsocket->send(&d_to_send[0], d_to_send.size());
          }
          if(p_tcpsocket)
          {
            p_tcpsocket->send(&d_to_send[0], d_to_send.size());
          }

          d_pkt_n = (d_pkt_n + 1) % difi::VITA_PKT_MOD;
          d_current_buff_idx = 0;
          d_pcks_since_last_reference++;
          d_packet_count++;
        }
      }
      return noutput_items;
    }

    template <class T, class S>
    void difi_sink_cpp_impl<T, S>::process_tags(int noutput_items)
    {
      auto abs_offset = this->nitems_read(0);
      std::vector<tag_t> tags;
      this->get_tags_in_range(tags, 0, abs_offset, abs_offset + noutput_items);
      for(tag_t tag : tags)
      {
        if(pmt::eqv(d_context_key, tag.key))
        {
          auto raw = pmt::dict_ref(tag.value, pmt::intern("raw"), pmt::get_PMT_NIL());;
          std::vector<int8_t> to_send = pmt::s8vector_elements(raw);
          d_raw = to_send;
          if(p_udpsocket)
          {
            p_udpsocket->send(&to_send[0], to_send.size());
          }

          if(p_tcpsocket)
          {
            p_tcpsocket->send(&to_send[0], to_send.size());
          }

          std::copy(to_send.begin(), to_send.begin() + difi::DIFI_HEADER_SIZE, d_raw.begin());
          d_full = u_int32_t(pmt::to_long(pmt::dict_ref(tag.value, pmt::mp("full"), pmt::get_PMT_NIL())));
          d_frac = pmt::to_uint64(pmt::dict_ref(tag.value, pmt::mp("frac"), pmt::get_PMT_NIL()));
          d_stream_number = pmt::to_uint64(pmt::dict_ref(tag.value, pmt::mp("stream_num"), pmt::get_PMT_NIL()));
          d_pcks_since_last_reference = 0;
        }
        else if(pmt::eqv(d_pkt_n_key, tag.key))
        {
          d_pkt_n = (d_pkt_n + 1) % difi::VITA_PKT_MOD; // missed packet, so account for this
          d_full = u_int32_t(pmt::to_long(pmt::dict_ref(tag.value, pmt::mp("full"), pmt::get_PMT_NIL())));
          d_frac = pmt::to_uint64(pmt::dict_ref(tag.value, pmt::mp("frac"), pmt::get_PMT_NIL()));
          d_data_len = pmt::to_uint64(pmt::dict_ref(tag.value, pmt::mp("data_len"), pmt::get_PMT_NIL())) - difi::DIFI_HEADER_SIZE;
          if (d_data_len % sizeof(S) != 0)
          {
            GR_LOG_WARN(this->d_logger, "data len cannot fit an integer number of samples, something is misconfigured");
          }
          d_samples_per_packet = d_data_len / sizeof(S);
          d_out_buf.clear(); // clear the data since we missed samples, start fresh
          d_out_buf.resize(d_data_len);
          d_current_buff_idx = 0;
          d_time_adj = (double)d_samples_per_packet / d_full_samp;
          d_pcks_since_last_reference = 0;
        }
        else if(pmt::eqv(d_static_change_key, tag.key))
        {
          d_static_bits = pmt::to_uint64(tag.value);
        }
      }
    }
    template <class T, class S>
    void difi_sink_cpp_impl<T, S>::pack_data()
    {
      std::vector<int8_t> to_send(difi::DIFI_HEADER_SIZE + d_out_buf.size());
      u_int32_t full;
      u_int64_t frac;
      std::tie(full, frac) = add_frac_full();
      u_int32_t header = d_static_bits ^ d_pkt_n << 16 ^ (d_data_len + difi::DIFI_HEADER_SIZE) / 4;
      pack_u32(&d_to_send[0], header);
      std::copy(d_raw.begin() + 4, d_raw.begin() + 16, d_to_send.begin() + 4);
      pack_u32(&d_to_send[16], full);
      pack_u64(&d_to_send[20], frac);
      std::copy(d_out_buf.begin(), d_out_buf.end(), d_to_send.begin() + difi::DIFI_HEADER_SIZE);
    }

    template <class T, class S>
    void difi_sink_cpp_impl<T, S>::send_context()
    {
        if(d_context_packet_size == 108)// this check is a temporary work around for a non-compliant hardware device
        {
            u_int32_t full;
            u_int64_t frac;
            std::tie(full, frac) = add_frac_full();
            pack_u32(&d_context_raw[16], full);
            pack_u64(&d_context_raw[20], frac);
        }
        u_int32_t header = d_context_static_bits ^ d_context_packet_count << 16 ^ (d_context_packet_size / 4);
        pack_u32(&d_context_raw[0], header);
        if(p_udpsocket)
        {
          p_udpsocket->send((int8_t*)&d_context_raw[0],d_context_raw.size());
        }

        if(p_tcpsocket)
        {
          p_tcpsocket->send((int8_t*)&d_context_raw[0], d_context_raw.size());
        }
        d_context_packet_count = (d_context_packet_count + 1) % difi::VITA_PKT_MOD;
    }

    template <class T, class S>
    std::tuple<u_int32_t, u_int64_t> difi_sink_cpp_impl<T, S>::add_frac_full()
    {
      auto frac = d_frac;
      auto full = d_full;
      auto time_adj = d_time_adj * d_pcks_since_last_reference;
      frac += u_int64_t((time_adj - u_int64_t(time_adj)) * difi::PICO_CONVERSION);
      if(frac >= difi::PICO_CONVERSION)
      {
          frac = d_frac % difi::PICO_CONVERSION;
          full += 1;
      }
      full += u_int32_t(time_adj);
      return std::make_tuple(full, frac);
    }

    template class difi_sink_cpp<gr_complex, std::complex<int16_t>>;
    template class difi_sink_cpp<gr_complex, std::complex<int8_t>>;
    template class difi_sink_cpp<std::complex<int8_t>, std::complex<int16_t>>;
    template class difi_sink_cpp<std::complex<int8_t>, std::complex<int8_t>>;

  } /* namespace difi */
} /* namespace gr */
