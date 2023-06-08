#include "crude_neteq_impl.h"

#include "common_audio/signal_processing/include/signal_processing_library.h"
#include "neteq/expand.h"
#include "neteq/merge.h"
#include "neteq/audio_multi_vector.h"
#include "neteq/sync_buffer.h"
#include "neteq/background_noise.h"
#include "neteq/random_vector.h"
#include "neteq/post_decode_vad.h"
#include "neteq/audio_frame.h"
#include "neteq/tick_timer.h"
#include "neteq/accelerate.h"
#include "neteq/buffer_level_filter.h"
#include "neteq/normal.h"
#include "neteq/preemptive_expand.h"
#include "neteq/delay_peak_detector.h"
#include "neteq/delay_manager.h"
#include "neteq/packet_buffer.h"
#include "neteq/decision_logic.h"
#include "neteq/optional.hpp"
#include "rtc_base/sanitizer.h"

namespace webrtc {

namespace {
void SetAudioFrameActivityAndType(bool vad_enabled,
                                  CrudeNetEqImpl::OutputType type,
                                  AudioFrame::VADActivity last_vad_activity,
                                  AudioFrame* audio_frame) {
    switch (type) {
        case CrudeNetEqImpl::OutputType::kNormalSpeech: {
            audio_frame->speech_type_ = AudioFrame::kNormalSpeech;
            audio_frame->vad_activity_ = AudioFrame::kVadActive;
            break;
        }
        case CrudeNetEqImpl::OutputType::kVadPassive: {
            // This should only be reached if the VAD is enabled.
            RTC_DCHECK(vad_enabled);
            audio_frame->speech_type_ = AudioFrame::kNormalSpeech;
            audio_frame->vad_activity_ = AudioFrame::kVadPassive;
            break;
        }
        case CrudeNetEqImpl::OutputType::kCNG: {
            audio_frame->speech_type_ = AudioFrame::kCNG;
            audio_frame->vad_activity_ = AudioFrame::kVadPassive;
            break;
        }
        case CrudeNetEqImpl::OutputType::kPLC: {
            audio_frame->speech_type_ = AudioFrame::kPLC;
            audio_frame->vad_activity_ = last_vad_activity;
            break;
        }
        case CrudeNetEqImpl::OutputType::kPLCCNG: {
            audio_frame->speech_type_ = AudioFrame::kPLCCNG;
            audio_frame->vad_activity_ = AudioFrame::kVadPassive;
            break;
        }
        default:
        RTC_NOTREACHED();
    }

    if (!vad_enabled) {
        // Always set kVadUnknown when receive VAD is inactive.
        audio_frame->vad_activity_ = AudioFrame::kVadUnknown;
    }
}
}

CrudeNetEqImpl::CrudeNetEqImpl(const CrudeNetEq::Config& config, std::unique_ptr<AudioDecoder> &decoder):
    tick_timer_(new TickTimer),
    buffer_level_filter_(new BufferLevelFilter),
    delay_peak_detector_(new DelayPeakDetector(tick_timer_.get())),
    delay_manager_(new DelayManager(config.max_packets_in_buffer, delay_peak_detector_.get(), tick_timer_.get())),
    packet_buffer_(new PacketBuffer(config.max_packets_in_buffer)),
    accelerate_factory_(new AccelerateFactory),
    expand_factory_(new ExpandFactory),
    preemptive_expand_factory_(new PreemptiveExpandFactory),
    vad_(new PostDecodeVad()),
    last_mode_(kModeNormal),
    decoded_buffer_length_(kMaxFrameSize),
    decoded_buffer_(new int16_t[decoded_buffer_length_]),
    playout_timestamp_(0),
    new_codec_(false),
    timestamp_(0),
    ssrc_(0),
    first_packet_(true),
    enable_fast_accelerate_(config.enable_fast_accelerate),
    enable_muted_state_(config.enable_muted_state),
    decoder_(std::move(decoder))
{
    int fs = config.sample_rate_hz;
    if (fs != 8000 && fs != 16000 && fs != 32000 && fs != 48000) {
        // RTC_LOG(LS_ERROR) << "Sample rate " << fs << " Hz not supported. "
        //                 << "Changing to 8000 Hz.";
        fs = 8000;
    }
    delay_manager_->SetMaximumDelay(config.max_delay_ms);
    fs_hz_ = fs;
    fs_mult_ = fs / 8000;
    last_output_sample_rate_hz_ = fs;
    output_size_samples_ = static_cast<size_t>(kOutputSizeMs * 8 * fs_mult_);
    decoder_frame_length_ = 2 * output_size_samples_;
    WebRtcSpl_Init();
    SetSampleRateAndChannels(fs, 1);  // Default is 1 channel.
    RTC_DCHECK(!vad_->enabled());
    if (config.enable_post_decode_vad) {
        vad_->Enable();
    }
}

CrudeNetEqImpl::~CrudeNetEqImpl() = default;

int CrudeNetEqImpl::InsertPacket(const RTPHeader& rtp_header,
                rtc::ArrayView<const uint8_t> payload,
                uint32_t receive_timestamp)
{
    rtc::MsanCheckInitialized(payload);

    if (InsertPacketInternal(rtp_header, payload, receive_timestamp) != 0) {
        return kFail;
    }
    return kOK;
}

int CrudeNetEqImpl::GetAudio(AudioFrame* audio_frame, bool* muted)
{
    if (GetAudioInternal(audio_frame, muted) != 0) {
        return kFail;
    }
    RTC_DCHECK_EQ(
        audio_frame->sample_rate_hz_,
        rtc::dchecked_cast<int>(audio_frame->samples_per_channel_ * 100));
    RTC_DCHECK_EQ(*muted, audio_frame->muted());
    SetAudioFrameActivityAndType(vad_->enabled(), LastOutputType(),
                                last_vad_activity_, audio_frame);
    last_vad_activity_ = audio_frame->vad_activity_;
    last_output_sample_rate_hz_ = audio_frame->sample_rate_hz_;
    RTC_DCHECK(last_output_sample_rate_hz_ == 8000 ||
                last_output_sample_rate_hz_ == 16000 ||
                last_output_sample_rate_hz_ == 32000 ||
                last_output_sample_rate_hz_ == 48000);
    return kOK;
}


int CrudeNetEqImpl::InsertPacketInternal(const RTPHeader& rtp_header,
                        rtc::ArrayView<const uint8_t> payload,
                        uint32_t receive_timestamp)
{
    if (payload.empty()){
        return kInvalidPointer;
    }

    Packet packet;
    packet.payload_type = rtp_header.payloadType;
    packet.sequence_number = rtp_header.sequenceNumber;
    packet.timestamp = rtp_header.timestamp;
    packet.payload.SetData(payload.data(), payload.size());

    // Store these for later use, since the first packet may very well disappear
    // before we need these values.
    uint32_t main_timestamp = packet.timestamp;
    uint8_t main_payload_type = packet.payload_type;
    uint16_t main_sequence_number = packet.sequence_number;

    bool update_sample_rate_and_channels = first_packet_;

    if (update_sample_rate_and_channels){

        packet_buffer_->Flush();

        sync_buffer_->IncreaseEndTimestamp(main_timestamp - timestamp_);

        timestamp_ = main_timestamp;
    }

    const int ret = packet_buffer_->InsertPacket(std::move(packet));
    if (ret == PacketBuffer::kFlushed) {
        // Reset DSP timestamp etc. if packet buffer flushed.
        update_sample_rate_and_channels = true;
    } else if (ret != PacketBuffer::kOK) {
        return kOtherError;
    }

    if (first_packet_) {
        first_packet_ = false;
        new_codec_ = true;
    }

    delay_manager_->LastDecodedWasCngOrDtmf(0);
    if (delay_manager_->last_pack_cng_or_dtmf() == 0) {
        const int number_of_primary_packets = 1;
        if (number_of_primary_packets > 0) {
            const size_t packet_length_samples =
                number_of_primary_packets * decoder_frame_length_;
            if (packet_length_samples != decision_logic_->packet_length_samples()) {
                decision_logic_->set_packet_length_samples(packet_length_samples);
                delay_manager_->SetPacketAudioLength(
                    rtc::dchecked_cast<int>((1000 * packet_length_samples) / fs_hz_));
            }
        }

        if ((int32_t)(main_timestamp - timestamp_) >= 0 && !new_codec_) {
            // Only update statistics if incoming packet is not older than last played
            // out packet, and if new codec flag is not set.
            delay_manager_->Update(main_sequence_number, main_timestamp, fs_hz_);
        }
    } else if (delay_manager_->last_pack_cng_or_dtmf() == -1) {
        // This is first "normal" packet after CNG or DTMF.
        // Reset packet time counter and measure time until next packet,
        // but don't update statistics.
        delay_manager_->set_last_pack_cng_or_dtmf(0);
        delay_manager_->ResetPacketIatCount();
    }

    return 0;
}

int CrudeNetEqImpl::GetAudioInternal(AudioFrame* audio_frame, bool* muted)
{
    Operations operation;
    PacketList packet_list;

    tick_timer_->Increment();

    if (enable_muted_state_ && expand_->Muted() && packet_buffer_->Empty()) {
        RTC_DCHECK_EQ(last_mode_, kModeExpand);
        audio_frame->Reset();
        RTC_DCHECK(audio_frame->muted());  // Reset() should mute the frame.
        playout_timestamp_ += static_cast<uint32_t>(output_size_samples_);
        audio_frame->sample_rate_hz_ = fs_hz_;
        audio_frame->samples_per_channel_ = output_size_samples_;
        audio_frame->timestamp_ =
            first_packet_
                ? 0
                : (playout_timestamp_) - static_cast<uint32_t>(audio_frame->samples_per_channel_);
        audio_frame->num_channels_ = sync_buffer_->Channels();
        *muted = true;
        return 0;
    }

    int return_value = GetDecision(&operation, &packet_list);
    if (return_value != 0) {
        last_mode_ = kModeError;
        return return_value;
    }

    AudioDecoder::SpeechType speech_type;
    bool play_dtmf = false;
    int length = 0;

    int decode_return_value = Decode(&packet_list, &operation, &length, &speech_type);

    algorithm_buffer_->Clear();
    switch (operation) {
        case kNormal: {
            DoNormal(decoded_buffer_.get(), length, speech_type, play_dtmf);
            break;
        }
        case kMerge: {
            DoMerge(decoded_buffer_.get(), length, speech_type, play_dtmf);
            break;
        }
        case kExpand: {
            RTC_DCHECK_EQ(return_value, 0);
            // if (!current_rtp_payload_type_ || !DoCodecPlc()) {
                return_value = DoExpand(play_dtmf);
            // }
            RTC_DCHECK_GE(sync_buffer_->FutureLength() - expand_->overlap_length(),
                            output_size_samples_);
            break;
        }
        case kAccelerate:
        case kFastAccelerate: {
            const bool fast_accelerate =
                enable_fast_accelerate_ && (operation == kFastAccelerate);
            return_value = DoAccelerate(decoded_buffer_.get(), length, speech_type,
                                        play_dtmf, fast_accelerate);
            break;
        }
        case kPreemptiveExpand: {
            return_value = DoPreemptiveExpand(decoded_buffer_.get(), length,
                                                speech_type, play_dtmf);
            break;
        }
        // case kRfc3389Cng:
        // case kRfc3389CngNoPacket: {
        //     return_value = DoRfc3389Cng(&packet_list, play_dtmf);
        //     break;
        // }
        // case kCodecInternalCng: {
        //     // This handles the case when there is no transmission and the decoder
        //     // should produce internal comfort noise.
        //     // TODO(hlundin): Write test for codec-internal CNG.
        //     DoCodecInternalCng(decoded_buffer_.get(), length);
        //     break;
        // }
        // case kDtmf: {
        //     // TODO(hlundin): Write test for this.
        //     return_value = DoDtmf(dtmf_event, &play_dtmf);
        //     break;
        // }
        case kUndefined: {
            assert(false);
            last_mode_ = kModeError;
            return kInvalidOperation;
        }
    }

    last_operation_ = operation;
    if (return_value < 0) {
        return return_value;
    }

    // Copy from |algorithm_buffer| to |sync_buffer_|.
    sync_buffer_->PushBack(*algorithm_buffer_);

    // Extract data from |sync_buffer_| to |output|.
    size_t num_output_samples_per_channel = output_size_samples_;
    size_t num_output_samples = output_size_samples_ * sync_buffer_->Channels();
    if (num_output_samples > AudioFrame::kMaxDataSizeSamples) {
        num_output_samples = AudioFrame::kMaxDataSizeSamples;
        num_output_samples_per_channel =
            AudioFrame::kMaxDataSizeSamples / sync_buffer_->Channels();
    }
    sync_buffer_->GetNextAudioInterleaved(num_output_samples_per_channel,
                                            audio_frame);
    audio_frame->sample_rate_hz_ = fs_hz_;

    if (sync_buffer_->FutureLength() < expand_->overlap_length()) {
        // The sync buffer should always contain |overlap_length| samples, but now
        // too many samples have been extracted. Reinstall the |overlap_length|
        // lookahead by moving the index.
        const size_t missing_lookahead_samples =
            expand_->overlap_length() - sync_buffer_->FutureLength();
        RTC_DCHECK_GE(sync_buffer_->next_index(), missing_lookahead_samples);
        sync_buffer_->set_next_index(sync_buffer_->next_index() -
                                    missing_lookahead_samples);
    }

    if (audio_frame->samples_per_channel_ != output_size_samples_) {
        audio_frame->Mute();
        return kSampleUnderrun;
    }

    // Update the background noise parameters if last operation wrote data
    // straight from the decoder to the |sync_buffer_|. That is, none of the
    // operations that modify the signal can be followed by a parameter update.
    if ((last_mode_ == kModeNormal) || (last_mode_ == kModeAccelerateFail) ||
        (last_mode_ == kModePreemptiveExpandFail) ||
        (last_mode_ == kModeRfc3389Cng) ||
        (last_mode_ == kModeCodecInternalCng)) {
        background_noise_->Update(*sync_buffer_, *vad_.get());
    }

    if (last_mode_ != kModeExpand && last_mode_ != kModeCodecPlc) {
        // If last operation was not expand, calculate the |playout_timestamp_| from
        // the |sync_buffer_|. However, do not update the |playout_timestamp_| if it
        // would be moved "backwards".
        uint32_t temp_timestamp =
            sync_buffer_->end_timestamp() -
            static_cast<uint32_t>(sync_buffer_->FutureLength());
        if (static_cast<int32_t>(temp_timestamp - playout_timestamp_) > 0) {
            playout_timestamp_ = temp_timestamp;
        }
    } else {
        // Use dead reckoning to estimate the |playout_timestamp_|.
        playout_timestamp_ += static_cast<uint32_t>(output_size_samples_);
    }

    audio_frame->timestamp_ =
        first_packet_
            ? 0
            : playout_timestamp_ - static_cast<uint32_t>(audio_frame->samples_per_channel_);


    if (decode_return_value)
        return decode_return_value;
    return return_value;
}

int CrudeNetEqImpl::GetDecision(Operations* operation, PacketList* packet_list)
{
    uint32_t end_timestamp = sync_buffer_->end_timestamp();
    if (!new_codec_) {
        const uint32_t five_seconds_samples = 5 * fs_hz_;
        packet_buffer_->DiscardOldPackets(end_timestamp, five_seconds_samples);
    }

    const Packet* packet = packet_buffer_->PeekNextPacket();

    uint64_t generated_noise_samples = 0;

    const int samples_left = static_cast<int>(sync_buffer_->FutureLength() -
                                                expand_->overlap_length());
    if (last_mode_ == kModeAccelerateSuccess ||
        last_mode_ == kModeAccelerateLowEnergy ||
        last_mode_ == kModePreemptiveExpandSuccess ||
        last_mode_ == kModePreemptiveExpandLowEnergy) {
        // Subtract (samples_left + output_size_samples_) from sampleMemory.
        decision_logic_->AddSampleMemory(
            -(samples_left + rtc::dchecked_cast<int>(output_size_samples_)));
    }

    *operation = decision_logic_->GetDecision(
        *sync_buffer_, *expand_, decoder_frame_length_, packet, last_mode_,
        false, generated_noise_samples, &reset_decoder_);

    // Check if we already have enough samples in the |sync_buffer_|. If so,
    // change decision to normal, unless the decision was merge, accelerate, or
    // preemptive expand.
    if (samples_left >= rtc::dchecked_cast<int>(output_size_samples_) &&
        *operation != kMerge && *operation != kAccelerate &&
        *operation != kFastAccelerate && *operation != kPreemptiveExpand) {
        *operation = kNormal;
        return 0;
    }

    decision_logic_->ExpandDecision(*operation);

    // Check conditions for reset.
    if (new_codec_ || *operation == kUndefined) {
        // The only valid reason to get kUndefined is that new_codec_ is set.
        assert(new_codec_);

        timestamp_ = packet->timestamp;
        *operation = kNormal;

        // Adjust |sync_buffer_| timestamp before setting |end_timestamp| to the
        // new value.
        sync_buffer_->IncreaseEndTimestamp(timestamp_ - end_timestamp);
        end_timestamp = timestamp_;
        new_codec_ = false;
        buffer_level_filter_->Reset();
        delay_manager_->Reset();
    }

    size_t required_samples = output_size_samples_;
    const size_t samples_10_ms = static_cast<size_t>(80 * fs_mult_);
    const size_t samples_20_ms = 2 * samples_10_ms;
    const size_t samples_30_ms = 3 * samples_10_ms;

    switch (*operation) {
        case kExpand: {
            timestamp_ = end_timestamp;
            return 0;
        }
        case kRfc3389CngNoPacket:
        case kCodecInternalCng: {
            return 0;
        }
        case kDtmf: {
            return 0;
        }
        case kAccelerate:
        case kFastAccelerate: {
            // In order to do an accelerate we need at least 30 ms of audio data.
            if (samples_left >= static_cast<int>(samples_30_ms)) {
                // Already have enough data, so we do not need to extract any more.
                decision_logic_->set_sample_memory(samples_left);
                decision_logic_->set_prev_time_scale(true);
                return 0;
            } else if (samples_left >= static_cast<int>(samples_10_ms) &&
                        decoder_frame_length_ >= samples_30_ms) {
                // Avoid decoding more data as it might overflow the playout buffer.
                *operation = kNormal;
                return 0;
            } else if (samples_left < static_cast<int>(samples_20_ms) &&
                        decoder_frame_length_ < samples_30_ms) {
                // Build up decoded data by decoding at least 20 ms of audio data. Do
                // not perform accelerate yet, but wait until we only need to do one
                // decoding.
                required_samples = 2 * output_size_samples_;
                *operation = kNormal;
            }
            // If none of the above is true, we have one of two possible situations:
            // (1) 20 ms <= samples_left < 30 ms and decoder_frame_length_ < 30 ms; or
            // (2) samples_left < 10 ms and decoder_frame_length_ >= 30 ms.
            // In either case, we move on with the accelerate decision, and decode one
            // frame now.
            break;
        }
        case kPreemptiveExpand: {
            // In order to do a preemptive expand we need at least 30 ms of decoded
            // audio data.
            if ((samples_left >= static_cast<int>(samples_30_ms)) ||
                (samples_left >= static_cast<int>(samples_10_ms) &&
                decoder_frame_length_ >= samples_30_ms)) {
                // Already have enough data, so we do not need to extract any more.
                // Or, avoid decoding more data as it might overflow the playout buffer.
                // Still try preemptive expand, though.
                decision_logic_->set_sample_memory(samples_left);
                decision_logic_->set_prev_time_scale(true);
                return 0;
            }
            if (samples_left < static_cast<int>(samples_20_ms) &&
                decoder_frame_length_ < samples_30_ms) {
                // Build up decoded data by decoding at least 20 ms of audio data.
                // Still try to perform preemptive expand.
                required_samples = 2 * output_size_samples_;
            }
            break;
        }
        case kMerge: {
            required_samples =
                std::max(merge_->RequiredFutureSamples(), required_samples);
            break;
        }
        default: {
        }
    }

    int extracted_samples = 0;
    if (packet) {
        sync_buffer_->IncreaseEndTimestamp(packet->timestamp - end_timestamp);

        extracted_samples = ExtractPackets(required_samples, packet_list);
        if (extracted_samples < 0) {
            return kPacketBufferCorruption;
        }
    }

    if (*operation == kAccelerate || *operation == kFastAccelerate ||
        *operation == kPreemptiveExpand) {
        decision_logic_->set_sample_memory(samples_left + extracted_samples);
        decision_logic_->set_prev_time_scale(true);
    }

    if (*operation == kAccelerate || *operation == kFastAccelerate) {
        // Check that we have enough data (30ms) to do accelerate.
        if (extracted_samples + samples_left < static_cast<int>(samples_30_ms)) {
            *operation = kNormal;
        }
    }

    timestamp_ = end_timestamp;

    return 0;
}

int CrudeNetEqImpl::Decode(PacketList* packet_list,
            Operations* operation,
            int* decoded_length,
            AudioDecoder::SpeechType* speech_type)
{
    *speech_type = AudioDecoder::kSpeech;

    *decoded_length = 0;

    int return_value;

    return_value = DecodeLoop(packet_list, *operation, decoded_length,
                              speech_type);

    if (*decoded_length < 0) {
        *decoded_length = 0;
        sync_buffer_->IncreaseEndTimestamp(
            static_cast<uint32_t>(decoder_frame_length_));

        *operation = kExpand;
    }
    if (*speech_type != AudioDecoder::kComfortNoise) {
        sync_buffer_->IncreaseEndTimestamp(
            *decoded_length / static_cast<int>(sync_buffer_->Channels()));
    }

    return return_value;
}

int CrudeNetEqImpl::DecodeLoop(PacketList* packet_list,
                const Operations& operation,
                int* decoded_length,
                AudioDecoder::SpeechType* speech_type)
{
    while (!packet_list->empty()) {

        assert(operation == kNormal || operation == kAccelerate ||
            operation == kFastAccelerate || operation == kMerge ||
            operation == kPreemptiveExpand);

        *decoded_length += decoder_->Decode(packet_list->front().payload.data(), 
                                            packet_list->front().payload.size(), 
                                            fs_hz_, decoder_frame_length_, 
                                            (int16_t *)&decoded_buffer_[*decoded_length], speech_type);

        decoder_frame_length_ = packet_list->front().payload.size();

        packet_list->pop_front();
        
        if (*decoded_length > rtc::dchecked_cast<int>(decoded_buffer_length_)) {
            packet_list->clear();
            return kDecodedTooMuch;
        }
    }
    
    return 0;
}

void CrudeNetEqImpl::DoNormal(const int16_t* decoded_buffer,
                size_t decoded_length,
                AudioDecoder::SpeechType speech_type,
                bool play_dtmf)
{
    assert(normal_.get());
    normal_->Process(decoded_buffer, decoded_length, last_mode_, algorithm_buffer_.get());

    if (decoded_length != 0) {
        last_mode_ = kModeNormal;
    }
}

void CrudeNetEqImpl::DoMerge(int16_t* decoded_buffer,
            size_t decoded_length,
            AudioDecoder::SpeechType speech_type,
            bool play_dtmf)
{
    assert(merge_.get());

    size_t new_length = merge_->Process(decoded_buffer, decoded_length, algorithm_buffer_.get());

    int expand_length_correction =
        rtc::dchecked_cast<int>(new_length) -
        rtc::dchecked_cast<int>(decoded_length / algorithm_buffer_->Channels());

    last_mode_ = kModeMerge;

    expand_->Reset();
}


int CrudeNetEqImpl::DoExpand(bool play_dtmf)
{
    while ((sync_buffer_->FutureLength() - expand_->overlap_length()) <
            output_size_samples_) {
        algorithm_buffer_->Clear();
        int return_value = expand_->Process(algorithm_buffer_.get());

        last_mode_ = kModeExpand;

        if (return_value < 0) {
            return return_value;
        }

        sync_buffer_->PushBack(*algorithm_buffer_);
        algorithm_buffer_->Clear();
    }

    return 0;
}

int CrudeNetEqImpl::DoAccelerate(int16_t* decoded_buffer,
                size_t decoded_length,
                AudioDecoder::SpeechType speech_type,
                bool play_dtmf,
                bool fast_accelerate)
{
    const size_t required_samples =
        static_cast<size_t>(240 * fs_mult_);  // Must have 30 ms.
    size_t borrowed_samples_per_channel = 0;
    size_t num_channels = algorithm_buffer_->Channels();
    size_t decoded_length_per_channel = decoded_length / num_channels;
    if (decoded_length_per_channel < required_samples) {
        // Must move data from the |sync_buffer_| in order to get 30 ms.
        borrowed_samples_per_channel =
            static_cast<int>(required_samples - decoded_length_per_channel);
        memmove(&decoded_buffer[borrowed_samples_per_channel * num_channels],
                decoded_buffer, sizeof(int16_t) * decoded_length);
        sync_buffer_->ReadInterleavedFromEnd(borrowed_samples_per_channel,
                                            decoded_buffer);
        decoded_length = required_samples * num_channels;
    }

    size_t samples_removed;
    Accelerate::ReturnCodes return_code =
        accelerate_->Process(decoded_buffer, decoded_length, fast_accelerate,
                            algorithm_buffer_.get(), &samples_removed);

    switch (return_code) {
        case Accelerate::kSuccess:
            last_mode_ = kModeAccelerateSuccess;
            break;
        case Accelerate::kSuccessLowEnergy:
            last_mode_ = kModeAccelerateLowEnergy;
            break;
        case Accelerate::kNoStretch:
            last_mode_ = kModeAccelerateFail;
            break;
        case Accelerate::kError:
            last_mode_ = kModeAccelerateFail;
            return kAccelerateError;
    }

    if (borrowed_samples_per_channel > 0) {
        size_t length = algorithm_buffer_->Size();
        if (length < borrowed_samples_per_channel) {
            sync_buffer_->ReplaceAtIndex(
                *algorithm_buffer_,
                sync_buffer_->Size() - borrowed_samples_per_channel);
            sync_buffer_->PushFrontZeros(borrowed_samples_per_channel - length);
            algorithm_buffer_->PopFront(length);
            assert(algorithm_buffer_->Empty());
        } else {
            sync_buffer_->ReplaceAtIndex(
                *algorithm_buffer_, borrowed_samples_per_channel,
                sync_buffer_->Size() - borrowed_samples_per_channel);
            algorithm_buffer_->PopFront(borrowed_samples_per_channel);
        }
    }

    expand_->Reset();
    return 0;
}

int CrudeNetEqImpl::DoPreemptiveExpand(int16_t* decoded_buffer,
                        size_t decoded_length,
                        AudioDecoder::SpeechType speech_type,
                        bool play_dtmf)
{
    const size_t required_samples =
        static_cast<size_t>(240 * fs_mult_);  // Must have 30 ms.
    size_t num_channels = algorithm_buffer_->Channels();
    size_t borrowed_samples_per_channel = 0;
    size_t old_borrowed_samples_per_channel = 0;
    size_t decoded_length_per_channel = decoded_length / num_channels;
    if (decoded_length_per_channel < required_samples) {
        // Must move data from the |sync_buffer_| in order to get 30 ms.
        borrowed_samples_per_channel =
            required_samples - decoded_length_per_channel;
        // Calculate how many of these were already played out.
        old_borrowed_samples_per_channel =
            (borrowed_samples_per_channel > sync_buffer_->FutureLength())
                ? (borrowed_samples_per_channel - sync_buffer_->FutureLength())
                : 0;
        memmove(&decoded_buffer[borrowed_samples_per_channel * num_channels],
                decoded_buffer, sizeof(int16_t) * decoded_length);
        sync_buffer_->ReadInterleavedFromEnd(borrowed_samples_per_channel,
                                            decoded_buffer);
        decoded_length = required_samples * num_channels;
    }

    size_t samples_added;
    PreemptiveExpand::ReturnCodes return_code = preemptive_expand_->Process(
        decoded_buffer, decoded_length, old_borrowed_samples_per_channel,
        algorithm_buffer_.get(), &samples_added);

    switch (return_code) {
        case PreemptiveExpand::kSuccess:
            last_mode_ = kModePreemptiveExpandSuccess;
            break;
        case PreemptiveExpand::kSuccessLowEnergy:
            last_mode_ = kModePreemptiveExpandLowEnergy;
            break;
        case PreemptiveExpand::kNoStretch:
            last_mode_ = kModePreemptiveExpandFail;
            break;
        case PreemptiveExpand::kError:
            last_mode_ = kModePreemptiveExpandFail;
            return kPreemptiveExpandError;
    }

    if (borrowed_samples_per_channel > 0) {
        sync_buffer_->ReplaceAtIndex(
            *algorithm_buffer_, borrowed_samples_per_channel,
            sync_buffer_->Size() - borrowed_samples_per_channel);
        algorithm_buffer_->PopFront(borrowed_samples_per_channel);
    }

    expand_->Reset();
    return 0;
}

int CrudeNetEqImpl::ExtractPackets(size_t required_samples, PacketList* packet_list)
{
    bool first_packet = true;
    uint8_t prev_payload_type = 0;
    uint32_t prev_timestamp = 0;
    uint16_t prev_sequence_number = 0;
    bool next_packet_available = false;

    const Packet* next_packet = packet_buffer_->PeekNextPacket();
    RTC_DCHECK(next_packet);
    if (!next_packet) {
        return -1;
    }
    uint32_t first_timestamp = next_packet->timestamp;
    size_t extracted_samples = 0;

    do {
        timestamp_ = next_packet->timestamp;
        tl::optional<Packet> packet = packet_buffer_->GetNextPacket();

        next_packet = nullptr;
        if (!packet) {
            assert(false);
            return -1;
        }

        if (first_packet) {
            first_packet = false;
            prev_sequence_number = packet->sequence_number;
            prev_timestamp = packet->timestamp;
            prev_payload_type = packet->payload_type;
        }
        const bool has_cng_packet = false;

        size_t packet_duration = 0;
        if (packet_duration == 0) {
            packet_duration = decoder_frame_length_;
        }
        extracted_samples = packet->timestamp - first_timestamp + packet_duration;

        packet_list->push_back(std::move(*packet));
        packet = tl::nullopt;

        next_packet = packet_buffer_->PeekNextPacket();
        next_packet_available = false;
        if (next_packet && prev_payload_type == next_packet->payload_type &&
            !has_cng_packet) {
            int16_t seq_no_diff = next_packet->sequence_number - prev_sequence_number;
            size_t ts_diff = next_packet->timestamp - prev_timestamp;
            if (seq_no_diff == 1 ||
                (seq_no_diff == 0 && ts_diff == decoder_frame_length_)) {
                    next_packet_available = true;
            }
            prev_sequence_number = next_packet->sequence_number;
        }
    } while (extracted_samples < required_samples && next_packet_available);

    if (extracted_samples > 0) {
        packet_buffer_->DiscardAllOldPackets(timestamp_);
    }

    return rtc::dchecked_cast<int>(extracted_samples);
}

void CrudeNetEqImpl::SetSampleRateAndChannels(int fs_hz, size_t channels)
{
    assert(fs_hz == 8000 || fs_hz == 16000 || fs_hz == 32000 || fs_hz == 48000);
    assert(channels > 0);

    fs_hz_ = fs_hz;
    fs_mult_ = fs_hz / 8000;
    output_size_samples_ = static_cast<size_t>(kOutputSizeMs * 8 * fs_mult_);
    decoder_frame_length_ = 2 * output_size_samples_;  // Initialize to 30ms.

    last_mode_ = kModeNormal;

    assert(vad_.get());
    vad_->Init();

    // Delete algorithm buffer and create a new one.
    algorithm_buffer_.reset(new AudioMultiVector(channels));

    // Delete sync buffer and create a new one.
    sync_buffer_.reset(new SyncBuffer(channels, kSyncBufferSize * fs_mult_));

    // Delete BackgroundNoise object and create a new one.
    background_noise_.reset(new BackgroundNoise(channels));

    // Reset random vector.
    random_vector_.Reset();

    UpdatePlcComponents(fs_hz, channels);

    // Move index so that we create a small set of future samples (all 0).
    sync_buffer_->set_next_index(sync_buffer_->next_index() -
                                expand_->overlap_length());

    normal_.reset(new Normal(fs_hz, nullptr, *background_noise_,
                            expand_.get()));
    accelerate_.reset(
        accelerate_factory_->Create(fs_hz, channels, *background_noise_));
    preemptive_expand_.reset(preemptive_expand_factory_->Create(
        fs_hz, channels, *background_noise_, expand_->overlap_length()));

    // Verify that |decoded_buffer_| is long enough.
    if (decoded_buffer_length_ < kMaxFrameSize * channels) {
        // Reallocate to larger size.
        decoded_buffer_length_ = kMaxFrameSize * channels;
        decoded_buffer_.reset(new int16_t[decoded_buffer_length_]);
    }

    // Create DecisionLogic if it is not created yet, then communicate new sample
    // rate and output size to DecisionLogic object.
    if (!decision_logic_.get()) {
        CreateDecisionLogic();
    }
    decision_logic_->SetSampleRate(fs_hz_, output_size_samples_);
}

CrudeNetEqImpl::OutputType CrudeNetEqImpl::LastOutputType()
{
    assert(vad_.get());
    assert(expand_.get());
    if (last_mode_ == kModeCodecInternalCng || last_mode_ == kModeRfc3389Cng) {
        return OutputType::kCNG;
    } else if (last_mode_ == kModeExpand && expand_->MuteFactor(0) == 0) {
        // Expand mode has faded down to background noise only (very long expand).
        return OutputType::kPLCCNG;
    } else if (last_mode_ == kModeExpand) {
        return OutputType::kPLC;
    } else if (vad_->running() && !vad_->active_speech()) {
        return OutputType::kVadPassive;
    } else {
        return OutputType::kNormalSpeech;
    }
}

void CrudeNetEqImpl::UpdatePlcComponents(int fs_hz, size_t channels)
{
    expand_.reset(expand_factory_->Create(background_noise_.get(),
                                            sync_buffer_.get(), &random_vector_,
                                            fs_hz, channels));
    merge_.reset(new Merge(fs_hz, channels, expand_.get(), sync_buffer_.get()));
}

void CrudeNetEqImpl::CreateDecisionLogic()
{
    decision_logic_.reset(DecisionLogic::Create(
        fs_hz_, output_size_samples_, false,
        *packet_buffer_.get(), delay_manager_.get(),
        buffer_level_filter_.get(), tick_timer_.get()));
}

}