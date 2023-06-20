// -*- c++ -*- //
// Copyright (c) Microsoft Corporation.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#ifndef INCLUDED_DIFI_TIMESTAMP_SOURCE_CPP_H
#define INCLUDED_DIFI_TIMESTAMP_SOURCE_CPP_H

namespace gr {
  namespace difi {

    enum difi_timestamp_source {
      // Read DIFI timestamps from signal packets
      DIFI_TIMESTAMP_SIGNAL_PACKET = 0,
      // Read DIFI timestamps from system clock
      DIFI_TIMESTAMP_SYSTEM_CLOCK = 1,
      // Read integer seconds from system clock and fractional part from signal
      // packets (handling correctly the case when the two timestamps straddle a
      // second boundary).
      DIFI_TIMESTAMP_MIXED = 2,
    };

  } // namespace difi
} // namespace gr

#endif /* INCLUDED_DIFI_TIMESTAMP_SOURCE_CPP_H */
