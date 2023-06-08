
#ifndef CRUDE_NETEQ_IMPL_H_
#define CRUDE_NETEQ_IMPL_H_

#include <memory>
#include <string>
#include <vector>

#include "rtc_base/constructormagic.h"
#include "neteq/defines.h"
#include "neteq/tick_timer.h"
#include "neteq/audio_multi_vector.h"
#include "neteq/crude_neteq.h"
#include "neteq/random_vector.h"
#include "neteq/audio_frame.h"
#include "neteq/optional.hpp"
#include "neteq/packet.h"
#include "neteq/audio_decoder.h"
#include "rtc_base/constructormagic.h"
#include "neteq/audio_decoder.h"

namespace webrtc {

class Accelerate;
class BackgroundNoise;
class BufferLevelFilter;
// class ComfortNoise;
class DecisionLogic;
class DelayManager;
class DelayPeakDetector;
class Expand;
class Merge;
class Normal;
class PacketBuffer;
class PostDecodeVad;
class PreemptiveExpand;
class RandomVector;
class SyncBuffer;
class TimestampScaler;
struct AccelerateFactory;
struct ExpandFactory;
struct PreemptiveExpandFactory;

class CrudeNetEqImpl : public webrtc::CrudeNetEq {
public:
    enum class OutputType {
        kNormalSpeech,
        kPLC,
        kCNG,
        kPLCCNG,
        kVadPassive
    };
    enum ErrorCodes {
        kNoError = 0,
        kOtherError,
        kUnknownRtpPayloadType,
        kDecoderNotFound,
        kInvalidPointer,
        kAccelerateError,
        kPreemptiveExpandError,
        kComfortNoiseErrorCode,
        kDecoderErrorCode,
        kOtherDecoderError,
        kInvalidOperation,
        kDtmfParsingError,
        kDtmfInsertError,
        kSampleUnderrun,
        kDecodedTooMuch,
        kRedundancySplitError,
        kPacketBufferCorruption
    };

    CrudeNetEqImpl(const CrudeNetEq::Config& config, std::unique_ptr<AudioDecoder> &decoder);

    ~CrudeNetEqImpl() override;

    int InsertPacket(const RTPHeader& rtp_header,
                    rtc::ArrayView<const uint8_t> payload,
                    uint32_t receive_timestamp) override;

    int GetAudio(AudioFrame* audio_frame, bool* muted) override;

protected:
    static const int kOutputSizeMs = 10;
    static const size_t kMaxFrameSize = 5760;  // 120 ms @ 48 kHz.
    static const size_t kSyncBufferSize = kMaxFrameSize + 60 * 48;

    int InsertPacketInternal(const RTPHeader& rtp_header,
                            rtc::ArrayView<const uint8_t> payload,
                            uint32_t receive_timestamp);

    int GetAudioInternal(AudioFrame* audio_frame, bool* muted);

    int GetDecision(Operations* operation,
                    PacketList* packet_list);

    int Decode(PacketList* packet_list,
                Operations* operation,
                int* decoded_length,
                AudioDecoder::SpeechType* speech_type);

    int DecodeLoop(PacketList* packet_list,
                    const Operations& operation,
                    int* decoded_length,
                    AudioDecoder::SpeechType* speech_type);

    void DoNormal(const int16_t* decoded_buffer,
                    size_t decoded_length,
                    AudioDecoder::SpeechType speech_type,
                    bool play_dtmf);

    void DoMerge(int16_t* decoded_buffer,
                size_t decoded_length,
                AudioDecoder::SpeechType speech_type,
                bool play_dtmf);

    bool DoCodecPlc();

    int DoExpand(bool play_dtmf);

    int DoAccelerate(int16_t* decoded_buffer,
                    size_t decoded_length,
                    AudioDecoder::SpeechType speech_type,
                    bool play_dtmf,
                    bool fast_accelerate);

    int DoPreemptiveExpand(int16_t* decoded_buffer,
                            size_t decoded_length,
                            AudioDecoder::SpeechType speech_type,
                            bool play_dtmf);

    int ExtractPackets(size_t required_samples, PacketList* packet_list);

    void SetSampleRateAndChannels(int fs_hz, size_t channels);

     OutputType LastOutputType();

    void UpdatePlcComponents(int fs_hz, size_t channels);
    void CreateDecisionLogic();

    const std::unique_ptr<TickTimer> tick_timer_;
    const std::unique_ptr<BufferLevelFilter> buffer_level_filter_;
    const std::unique_ptr<DelayPeakDetector> delay_peak_detector_;
    const std::unique_ptr<DelayManager> delay_manager_;

    const std::unique_ptr<PacketBuffer> packet_buffer_;
    // const std::unique_ptr<TimestampScaler> timestamp_scaler_;
    const std::unique_ptr<PostDecodeVad> vad_;
    const std::unique_ptr<ExpandFactory> expand_factory_;
    const std::unique_ptr<AccelerateFactory> accelerate_factory_;
    const std::unique_ptr<PreemptiveExpandFactory> preemptive_expand_factory_;
    std::unique_ptr<BackgroundNoise> background_noise_;
    std::unique_ptr<DecisionLogic> decision_logic_;
    std::unique_ptr<AudioMultiVector> algorithm_buffer_;
    std::unique_ptr<SyncBuffer> sync_buffer_;
    std::unique_ptr<Expand> expand_;
    std::unique_ptr<Normal> normal_;
    std::unique_ptr<Merge> merge_;
    std::unique_ptr<Accelerate> accelerate_;
    std::unique_ptr<PreemptiveExpand> preemptive_expand_;
    RandomVector random_vector_;
    // std::unique_ptr<ComfortNoise> comfort_noise_;

    int fs_hz_;
    int fs_mult_;
    int last_output_sample_rate_hz_;
    size_t output_size_samples_;
    size_t decoder_frame_length_;
    Modes last_mode_;
    Operations last_operation_;
    size_t decoded_buffer_length_;
    std::unique_ptr<int16_t[]> decoded_buffer_;
    uint32_t playout_timestamp_;
    bool new_codec_;
    uint32_t timestamp_;
    bool reset_decoder_;
    uint32_t ssrc_;
    bool first_packet_;
    bool enable_fast_accelerate_;
    const bool enable_muted_state_;
    AudioFrame::VADActivity last_vad_activity_;

    std::unique_ptr<TickTimer::Stopwatch> generated_noise_stopwatch_;
    std::vector<uint32_t> last_decoded_timestamps_;

    std::unique_ptr<AudioDecoder> decoder_;

private:
    RTC_DISALLOW_COPY_AND_ASSIGN(CrudeNetEqImpl);
};

}  // namespace webrtc
#endif
