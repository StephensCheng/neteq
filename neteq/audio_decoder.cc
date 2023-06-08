#include "neteq/audio_decoder.h"
namespace webrtc {

int AudioDecoder::Decode(const uint8_t* encoded,
            size_t encoded_len,
            int sample_rate_hz,
            size_t max_decoded_bytes,
            int16_t* decoded,
            SpeechType* speech_type)
{
    return DecodeInternal(encoded, encoded_len, sample_rate_hz, decoded, speech_type);
}

int AudioDecoder::PacketDuration(const uint8_t* encoded, size_t encoded_len) const
{
    return kNotImplemented;
}
}

