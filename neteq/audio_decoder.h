#ifndef AUDIO_DECODER_H_
#define AUDIO_DECODER_H_

#include <memory>
#include <vector>

//#include "absl/types/optional.h"
#include "neteq/array_view.h"
#include "rtc_base/buffer.h"
#include "rtc_base/constructormagic.h"

namespace webrtc {

class AudioDecoder {
public:
    AudioDecoder() = default;
    virtual ~AudioDecoder() = default;

    enum SpeechType {
        kSpeech = 1,
        kComfortNoise = 2,
    };

    enum { kNotImplemented = -2 };

    int Decode(const uint8_t* encoded,
                size_t encoded_len,
                int sample_rate_hz,
                size_t max_decoded_bytes,
                int16_t* decoded,
                SpeechType* speech_type);
    virtual int PacketDuration(const uint8_t* encoded, size_t encoded_len) const;

    virtual void Reset() = 0;
    virtual size_t Channels() const = 0;
    virtual int SampleRateHz() const = 0;

protected:
    virtual int DecodeInternal(const uint8_t* encoded,
                                size_t encoded_len,
                                int sample_rate_hz,
                                int16_t* decoded,
                                SpeechType* speech_type) = 0;
private:
    RTC_DISALLOW_COPY_AND_ASSIGN(AudioDecoder);
};

}
#endif
