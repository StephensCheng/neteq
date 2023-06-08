#ifndef CRUDE_NETEQ_H_
#define CRUDE_NETEQ_H_

#include <string.h>  // Provide access to size_t.

#include <string>
#include <vector>

#include "rtc_base/constructormagic.h"
#include "neteq/rtp_header.h"
#include "neteq/array_view.h"
#include "neteq/audio_decoder.h"

namespace webrtc {

class AudioFrame;

// This is the interface class for NetEq.
class CrudeNetEq {
public:
    struct Config {
        Config();
        Config(const Config&);
        Config(Config&&);
        ~Config();
        Config& operator=(const Config&);
        Config& operator=(Config&&);

        std::string ToString() const;
        int sample_rate_hz = 16000;  // Initial value. Will change with input data.
        bool enable_post_decode_vad = false;
        size_t max_packets_in_buffer = 50;
        int max_delay_ms = 2000;
        bool enable_fast_accelerate = false;
        bool enable_muted_state = false;
    };

    enum ReturnCodes { kOK = 0, kFail = -1 };

    static CrudeNetEq* Create(
        const CrudeNetEq::Config& config, std::unique_ptr<AudioDecoder> &decoder);

    virtual ~CrudeNetEq() {}

    virtual int InsertPacket(const RTPHeader& rtp_header,
                            rtc::ArrayView<const uint8_t> payload,
                            uint32_t receive_timestamp) = 0;

    virtual int GetAudio(
        AudioFrame* audio_frame,
        bool* muted) = 0;

//   virtual void SetCodecs(const std::map<int, SdpAudioFormat>& codecs) = 0;
//   virtual bool SetMinimumDelay(int delay_ms) = 0;
//   virtual bool SetMaximumDelay(int delay_ms) = 0;
//   virtual int TargetDelayMs() const = 0;
//   virtual int CurrentDelayMs() const = 0;
//   virtual absl::optional<uint32_t> GetPlayoutTimestamp() const = 0;
//   virtual int SyncBufferSizeMs() const = 0;

protected:
    CrudeNetEq() {}

private:
    RTC_DISALLOW_COPY_AND_ASSIGN(CrudeNetEq);
};

}  // namespace webrtc
#endif  // MODULES_AUDIO_CODING_NETEQ_INCLUDE_NETEQ_H_
