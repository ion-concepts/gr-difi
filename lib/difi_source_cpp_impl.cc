// -*- c++ -*-
// Copyright (c) Microsoft Corporation and Welkin Sciences, LLC.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.

#include <gnuradio/io_signature.h>
#include <functional>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "difi_source_cpp_impl.h"
#include "tcp_server.h"
#include "udp_socket.h"

#define NUM_BYTES_PER_WORD (4)

namespace gr {
  namespace difi {

    u_int32_t unpack_u32(u_int8_t *start)
    {
      u_int32_t val;
      memcpy(&val, start, sizeof(val));
      return ntohl(val);
    }
    u_int64_t unpack_u64(u_int8_t *start)
    {
      u_int64_t val;
      memcpy(&val, start, sizeof(val));
      return be64toh(val);
    }

      u_int32_t unpack_u32(int8_t *start)
    {
      u_int32_t val;
      memcpy(&val, start, sizeof(val));
      return ntohl(val);
    }
    u_int64_t unpack_u64(int8_t *start)
    {
      u_int64_t val;
      memcpy(&val, start, sizeof(val));
      return be64toh(val);
    }

    template<>
    void difi_source_cpp_impl<gr_complex, std::complex<int8_t>>::unpack_samples(
                                                                                gr_complex* output_vector,
                                                                                const std::complex<int8_t>* input_vector,
                                                                                size_t num_samples)
    {
      volk_8i_s32f_convert_32f(reinterpret_cast<float*>(output_vector),
                               reinterpret_cast<const int8_t*>(input_vector),
                               128.0,
                               2 * num_samples);
    }

    template<>
    void difi_source_cpp_impl<gr_complex, std::complex<int16_t>>::unpack_samples(
                                                                                 gr_complex* output_vector,
                                                                                 const std::complex<int16_t>* input_vector,
                                                                                 size_t num_samples)
    {
      volk_16i_s32f_convert_32f(reinterpret_cast<float*>(output_vector),
                                reinterpret_cast<const int16_t*>(input_vector),
                                32768.0,
                                2 * num_samples);
    }

    template<>
    void difi_source_cpp_impl<std::complex<int8_t>, std::complex<int8_t>>::unpack_samples(
                                                                                          std::complex<int8_t>* output_vector,
                                                                                          const std::complex<int8_t>* input_vector,
                                                                                          size_t num_samples)
    {
      
      std::memcpy(output_vector, input_vector, 2 * num_samples);
    }

     template<>
     void difi_source_cpp_impl<std::complex<int8_t>, std::complex<int16_t>>::unpack_samples(
                                                                                            std::complex<int8_t>* output_vector,
                                                                                            const std::complex<int16_t>* input_vector,
                                                                                            size_t num_samples)
    {
      volk_16i_convert_8i(reinterpret_cast<int8_t*>(output_vector),
                          reinterpret_cast<const int16_t*>(input_vector),
                          2 * num_samples);
    }

    template <class T, class S>
    typename difi_source_cpp<T, S>::sptr difi_source_cpp<T, S>::make(std::string ip_addr, uint32_t port, uint8_t socket_type, int stream_number, int context_pkt_behavior)
    {
      return gnuradio::make_block_sptr<difi_source_cpp_impl<T, S>>(ip_addr, port, socket_type, stream_number, context_pkt_behavior);
    }

    template <class T, class S>
    difi_source_cpp_impl<T, S>::difi_source_cpp_impl(std::string ip_addr, uint32_t port, uint8_t socket_type, int stream_number, int context_pkt_behavior)
        : gr::sync_block("source_cpp", gr::io_signature::make(0, 0, 0),
                         gr::io_signature::make(1 /* min outputs */,
                                                1 /*max outputs */,
                                                sizeof(T))),
          d_stream_number(stream_number),
          d_behavior(context_pkt_behavior),
          d_send(true),
          d_context(NULL),
          d_last_pkt_n(-1),
          d_static_bits(-1),
          p_tcpserver(NULL),
          p_udpsocket(NULL)

    {

      socket_type = (socket_type == 1) ?  SOCK_STREAM : SOCK_DGRAM;

      if(socket_type == SOCK_STREAM)
      {
        p_tcpserver = new tcp_server(ip_addr,port);
      }
      else
      {
        p_udpsocket = new udp_socket(ip_addr,port,true);
      }

      d_packet_buffer.resize(9000);
      this->set_output_multiple(9000);

      this->message_port_register_out(pmt::intern("pck_n"));
    }

    template <class T, class S>
    difi_source_cpp_impl<T, S>::~difi_source_cpp_impl()
    {
      if(p_tcpserver)
        delete p_tcpserver;

      if(p_udpsocket)
        delete p_udpsocket;
    }

    template <class T, class S>
    int difi_source_cpp_impl<T, S>::work(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
    {
      T *out = reinterpret_cast<T *>(output_items[0]);
      return buffer_and_send(out, noutput_items);
    }

    template <class T, class S>
    int difi_source_cpp_impl<T, S>::buffer_and_send(T *out, int noutput_items)
    {
      boost::this_thread::disable_interruption disable_interrupt;
      int size_gotten = -1;

      if(p_tcpserver and p_tcpserver->is_client_connected())
      {
        // read the first word to determine the size of the packet
        size_gotten = p_tcpserver->read(&d_packet_buffer[0],NUM_BYTES_PER_WORD);

        if(size_gotten == NUM_BYTES_PER_WORD)
        {
            // compute the number of bytes per packet
            // per DIFI version 1.0.0 spec each packet contains the packet size (number of 32-bit words) in the header
            // Packet Size: Bits 0-15
            // check packet size
            auto header = unpack_u32(&d_packet_buffer[0]);
            int pkt_size = NUM_BYTES_PER_WORD * (header&0xffff);
            int32_t num_bytes_to_read = pkt_size - NUM_BYTES_PER_WORD;

            if(num_bytes_to_read > 0)
                size_gotten += p_tcpserver->read(&d_packet_buffer[NUM_BYTES_PER_WORD],num_bytes_to_read);
        }
      }
      else if(p_udpsocket)
      {
        size_gotten = p_udpsocket->read(&d_packet_buffer[0], d_packet_buffer.size());
      }

      if(size_gotten <= 0)
      {
        return 0;
      }

      if (size_gotten % sizeof(S) != 0)
      {
        GR_LOG_WARN(this->d_logger, "got a packet which is not divisible by the number bytes per sample, samples will be lost. Check your bit depth configuration.");
      }

      header_data header;
      parse_header(header);
      if (d_stream_number != -1 and header.stream_num != d_stream_number)
      {
        GR_LOG_WARN(this->d_logger, "got wrong stream number, " + std::to_string(header.stream_num) + " expected " + std::to_string(d_stream_number));
        return 0;
      }
      if (header.type == 1 and
          ((d_last_pkt_n != -1 and (d_last_pkt_n + 1) % VITA_PKT_MOD != header.pkt_n))
          or (d_last_pkt_n == -1 and header.type == 1))
      {
        if (d_last_pkt_n != -1) {
          GR_LOG_WARN(this->d_logger, "got an out of order packet, " + std::to_string(header.pkt_n) +  " expected " + std::to_string((d_last_pkt_n + 1) % VITA_PKT_MOD));
        }
        const auto pck_n_dict = make_pkt_n_dict(header.pkt_n, size_gotten);
        const auto offset = this->nitems_written(0);
        this->add_item_tag(0, offset, pmt::intern("pck_n"), pck_n_dict);
        const auto pck_n_msg = pmt::dict_add(pck_n_dict, pmt::intern("offset"), pmt::from_uint64(offset));
        this->message_port_pub(pmt::intern("pck_n"), pck_n_msg);
      }

      if (header.type == 1 and d_send) // one is a data packet (see DIFI spec)
      {
        const int out_items = std::min((size_gotten - static_cast<int>(difi::DATA_START_IDX)) / sizeof(S),
                                       static_cast<size_t>(noutput_items));
        d_last_pkt_n = header.pkt_n;
        if (d_context != NULL)
        {
          this->add_item_tag(0, this->nitems_written(0) + d_deque.size(), pmt::intern("context"), d_context);
          d_context = NULL;
        }

        unpack_samples(out, reinterpret_cast<const S*>(&d_packet_buffer[difi::DATA_START_IDX]), out_items);
        return out_items;
      }
      else
      {
        d_context = d_behavior == context_behavior::ignore ? NULL : make_context_dict(header, size_gotten);
        return d_send ? buffer_and_send(out, noutput_items) : 0;
      }
    }

    template <class T, class S>
    void difi_source_cpp_impl<T, S>::parse_header(header_data &data)
    {
      auto header = unpack_u32(&d_packet_buffer[0]);
      auto stream_number = unpack_u32(&d_packet_buffer[4]);
      auto full = unpack_u32(&d_packet_buffer[16]);
      auto frac = unpack_u64(&d_packet_buffer[20]);
      data.type = header >> 28;
      data.pkt_n = (header >> 16) & 0xf;
      data.header = header;
      data.stream_num = stream_number;
      int32_t static_part = header & 0xfff00000;
      if(data.type == 1)
      {
        d_last_frac = frac;
        d_last_full = full;
        if(d_static_bits != static_part)
        {
          d_static_bits = static_part;
          this->add_item_tag(0, this->nitems_written(0), pmt::intern("static_change"), pmt::from_uint64(static_part));
        }
      }
    }

    template <class T, class S>
    pmt::pmt_t difi_source_cpp_impl<T, S>::make_pkt_n_dict(int pkt_n, int size_gotten)
    {
      pmt::pmt_t dict = pmt::make_dict();
      auto full = unpack_u32(&d_packet_buffer[16]);
      auto frac = unpack_u64(&d_packet_buffer[20]);
      dict = pmt::dict_add(dict, pmt::intern("pck_n"), pmt::from_uint64((u_int64_t)pkt_n));
      dict = pmt::dict_add(dict, pmt::intern("data_len"), pmt::from_uint64(size_gotten));
      dict = pmt::dict_add(dict, pmt::intern("full"), pmt::from_long(full));
      dict = pmt::dict_add(dict, pmt::intern("frac"), pmt::from_uint64(frac));
      return dict;
    }

    template <class T, class S>
    pmt::pmt_t difi_source_cpp_impl<T, S>::make_context_dict(header_data &header, int size_gotten)
    {
      pmt::pmt_t pmt_dict = pmt::make_dict();
      context_packet context;
      if(size_gotten == 72)
      {
        unpack_context_alt(context);
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("header"), pmt::from_long(header.header));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("stream_num"), pmt::from_uint64(header.stream_num));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("class_id"), pmt::from_long(context.class_id));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("full"), pmt::from_long(d_last_full));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("frac"), pmt::from_uint64(d_last_frac));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("CIF"), pmt::from_long(context.cif));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("bandwidth"), pmt::from_double(context.bw));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_reference_frequency"), pmt::from_double(context.if_ref_freq));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("rf_reference_frequency"), pmt::from_double(context.rf_ref_freq));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_band_offset"), pmt::from_double(context.if_band_offset));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("samp_rate"), pmt::from_double(context.samp_rate));;
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("state_and_event_indicator"), pmt::from_long(context.state_indicators));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("data_packet_payload_format"), pmt::from_uint64(context.payload_format));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("raw"), pmt::init_s8vector(size_gotten, &d_packet_buffer[0]));
      }
      else if (size_gotten == 108)
      {
        unpack_context(context);
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("header"), pmt::from_long(header.header));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("stream_num"), pmt::from_uint64(header.stream_num));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("class_id"), pmt::from_long(context.class_id));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("full"), pmt::from_long(context.full));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("frac"), pmt::from_uint64(context.frac));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("CIF"), pmt::from_long(context.cif));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("reference_point"), pmt::from_long(context.ref_point));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("bandwidth"), pmt::from_double(context.bw));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_reference_frequency"), pmt::from_double(context.if_ref_freq));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("rf_reference_frequency"), pmt::from_double(context.rf_ref_freq));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_band_offset"), pmt::from_double(context.if_band_offset));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("reference_level"), pmt::from_float(context.ref_lvl));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("rf_gain"), pmt::from_float(context.rf_gain));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_gain"), pmt::from_float(context.if_gain));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("samp_rate"), pmt::from_double(context.samp_rate));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("timestamp_adjustment"), pmt::from_long(context.t_adj));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("timestamp_calibration_time"), pmt::from_uint64(context.t_cal));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("state_and_event_indicator"), pmt::from_long(context.state_indicators));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("data_packet_payload_format"), pmt::from_uint64(context.payload_format));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("raw"), pmt::init_s8vector(size_gotten, &d_packet_buffer[0]));
      }
      else if (size_gotten == 84)
      {
        // Non DIFI compliant context extension packet format used by Kratos SNNB in 1.7.5
        unpack_context_kratos_snnb(context);
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("header"), pmt::from_long(header.header));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("stream_num"), pmt::from_uint64(header.stream_num));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("class_id"), pmt::from_long(context.class_id));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("full"), pmt::from_long(context.full));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("frac"), pmt::from_uint64(context.frac));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("CIF"), pmt::from_long(context.cif));
        //pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("reference_point"), pmt::from_long(context.ref_point));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("bandwidth"), pmt::from_double(context.bw));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_reference_frequency"), pmt::from_uint64(context.if_ref_freq));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("rf_reference_frequency"), pmt::from_uint64(context.rf_ref_freq));
        //pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_band_offset"), pmt::from_uint64(context.if_band_offset));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("reference_level"), pmt::from_long(context.ref_lvl));
        // Not clear that this should be broken out into 2 different gains here if context extension packet is really VITA49.0
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("rf_gain"), pmt::from_float(context.rf_gain)); 
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("if_gain"), pmt::from_float(context.if_gain));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("samp_rate"), pmt::from_double(context.samp_rate));
        //pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("timestamp_adjustment"), pmt::from_uint64(context.t_adj));
        //pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("timestamp_calibration_time"), pmt::from_uint64(context.t_cal));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("state_and_event_indicator"), pmt::from_long(context.state_indicators));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("data_packet_payload_format"), pmt::from_uint64(context.payload_format));
        pmt_dict = pmt::dict_add(pmt_dict, pmt::intern("raw"), pmt::init_s8vector(size_gotten, &d_packet_buffer[0]));
      }
      else if (size_gotten == 44)
      {
        std::string error_string = "ignoring apparent version flow signal context packet";
        GR_LOG_WARN(this->d_logger, error_string);
      }
      auto r_bit_depth = (context.payload_format >> 32 & 0x0000001f) + 1;
      if (size_gotten != 44 and (r_bit_depth != sizeof(S) * 8 / 2 or (size_gotten != 108 and size_gotten != 72 and size_gotten != 84)))
      {
        std::string error_string = size_gotten == 108 or size_gotten == 72 ?
                                    "The context packet bit depth does not match the input bit depth, check your configuration.\nContext packet bit depth is: " + std::to_string(r_bit_depth) :
                                    "The context packet size is not 108 bits per DIFI spec. The context packet recieved size is: " + std::to_string(size_gotten);
        if (d_behavior == context_behavior::throw_exe){
            GR_LOG_ERROR(this->d_logger, error_string);
            throw std::runtime_error(error_string);
        }
        d_send = d_behavior == context_behavior::warnings_forward;
        GR_LOG_WARN(this->d_logger, error_string);
        return NULL;
      }
      d_send = true;
      return pmt_dict;
    }

    template <class T, class S>
    void difi_source_cpp_impl<T, S>::unpack_context_alt(context_packet &context)
    {
      int idx = 0;
      context.class_id = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]);
      context.cif = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]);
      context.bw = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]));
      context.if_ref_freq = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]));
      context.rf_ref_freq = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]));
      context.if_band_offset = parse_vita_fixed_double((int64_t)unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]));
      context.samp_rate = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]));
      context.state_indicators = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]);
      context.payload_format = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_ALT_OFFSETS[idx++]]);
    }
    template <class T, class S>
    void difi_source_cpp_impl<T, S>::unpack_context(context_packet &context)
    {
      int idx = 0;
      context.class_id = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.full = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.frac = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.cif = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.ref_point = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.bw = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]));
      context.if_ref_freq = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]));
      context.rf_ref_freq = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]));
      context.if_band_offset = parse_vita_fixed_double((int64_t)unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]));
      context.ref_lvl = parse_vita_fixed_float((int16_t)(0xffffU & unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]])));
      u_int32_t gains = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.rf_gain = parse_vita_fixed_float((int16_t)(0xffffU & gains));
      context.if_gain = parse_vita_fixed_float((int16_t)(gains >> 16));
      context.samp_rate = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]));
      context.t_adj = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.t_cal = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.state_indicators = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
      context.payload_format = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_OFFSETS[idx++]]);
    }
    template <class T, class S>
    void difi_source_cpp_impl<T, S>::unpack_context_kratos_snnb(context_packet &context)
    {
      int idx = 0;
      context.class_id = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.full = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.frac = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.cif = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      //context.ref_point = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.bw = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]));
      context.if_ref_freq = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.rf_ref_freq = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      //context.if_band_offset = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.ref_lvl = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      // Not clear that this should be broken out into 2 different gains here if context extension packet is really VITA49.0
      u_int32_t gains = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.rf_gain = parse_vita_fixed_float((int16_t)(0xffffU & gains));
      context.if_gain = parse_vita_fixed_float((int16_t)(gains >> 16));
      context.samp_rate = parse_vita_fixed_double(unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]));
      //context.t_adj = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      //context.t_cal = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.state_indicators = unpack_u32(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx++]]);
      context.payload_format = unpack_u64(&d_packet_buffer[difi::CONTEXT_PACKET_KRATOS_SNNB_OFFSETS[idx]]);
    }

    template class difi_source_cpp<gr_complex, std::complex<int16_t>>;
    template class difi_source_cpp<gr_complex, std::complex<int8_t>>;
    template class difi_source_cpp<std::complex<int8_t>, std::complex<int16_t>>;
    template class difi_source_cpp<std::complex<int8_t>, std::complex<int8_t>>;
  } /* namespace difi */
} /* namespace gr */
