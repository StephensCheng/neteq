/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTP_HEADER_H_
#define RTP_HEADER_H_

#include <stddef.h>
#include <string.h>
#include <string>
#include <vector>

namespace webrtc {

// RTP
enum { kRtpCsrcSize = 15 };  // RFC 3550 page 13

struct RTPHeader {
    RTPHeader();
    RTPHeader(const RTPHeader& other);
    RTPHeader& operator=(const RTPHeader& other);

    bool markerBit;
    uint8_t payloadType;
    uint16_t sequenceNumber;
    uint32_t timestamp;
    uint32_t ssrc;
    uint8_t numCSRCs;
    uint32_t arrOfCSRCs[kRtpCsrcSize];
    size_t paddingLength;
    size_t headerLength;
    int payload_type_frequency;
//   RTPHeaderExtension extension;
};

}  // namespace webrtc

#endif  // API_RTP_HEADERS_H_
