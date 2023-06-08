
#include <stdio.h>
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
//#include "neteq/optional.hpp"
#include "neteq/rtp_header.h"
#include "neteq/crude_neteq.h"
#include "neteq/audio_decoder_pcm.h"
#include "codec/interface/g711/g711_interface.h"


using namespace webrtc;

int samples_10ms = static_cast<size_t>(10 * 8 * 6);
int samples_20ms = samples_10ms * 2;

CrudeNetEq *neteq_ = nullptr;

void init()
{
    CrudeNetEq::Config cfg;
    cfg.max_packets_in_buffer = 100;
    cfg.enable_fast_accelerate = true;
    cfg.enable_muted_state = false;
    cfg.sample_rate_hz = 8000;
    cfg.max_delay_ms = 1000;
    cfg.enable_post_decode_vad = true;

    samples_10ms = 10 * 8 * cfg.sample_rate_hz / 8000;
    samples_20ms = samples_10ms * 2;

    std::unique_ptr<AudioDecoder> decoder(new AudioDecoderPcmA(1));
    neteq_ = CrudeNetEq::Create(cfg, decoder);
}

size_t encode_pcm(const int16_t* speechIn, size_t len, uint8_t* encoded)
{
    return WebRtcG711_EncodeA(speechIn, len, encoded);
}

int main()
{
    init();

    const uint8_t kPayloadType = 0;
    uint16_t kFirstSequenceNumber = 0;
    uint32_t kFirstTimestamp = 0;
    const uint32_t kFirstReceiveTime = 17;

    FILE *pcm = fopen("MARVEL_V1.pcm", "rb");
    FILE *outfile = fopen("plc_V1.pcm", "wb");
    if(pcm == NULL || outfile == NULL){
        printf("open pcm or outfile file failed!\n");
        return 0;
    }

    int16_t buf[960];
    int read;

    while(!feof(pcm)){

        static int count = 0;
        static int total = 0;
        read = fread(buf, sizeof(int16_t), samples_20ms, pcm);
        if(read != samples_20ms){
            printf("read: %d\n", read);
            break;
        }

        RTPHeader rtp_header;
        rtp_header.payloadType = kPayloadType;
        rtp_header.sequenceNumber = kFirstSequenceNumber;
        rtp_header.timestamp = kFirstTimestamp;

        uint8_t speechIn[320] = {0};
        memcpy(speechIn, buf, 320);
        uint8_t *payload = new uint8_t[320];
        size_t payloadlen = encode_pcm((const int16_t *)speechIn, 160, payload);
        
        // InsertPacket(rtp_header, rtc::ArrayView<const uint8_t>(payload, 1920), kFirstReceiveTime);
        neteq_->InsertPacket(rtp_header, rtc::ArrayView<const uint8_t>(payload, payloadlen), kFirstReceiveTime);
        kFirstTimestamp += 160;
        kFirstSequenceNumber++;
        total++;
        if (count++ % 2 == 0 && total <= 60){
            
            read = fread(buf, sizeof(int16_t), samples_20ms, pcm);
            if(read != samples_20ms){
                printf("read: %d\n", read);
                break;
            }
            total++;

            RTPHeader rtp_header;
            rtp_header.payloadType = kPayloadType;
            rtp_header.sequenceNumber = kFirstSequenceNumber;
            rtp_header.timestamp = kFirstTimestamp;

            uint8_t speechIn[320] = {0};
            memcpy(speechIn, buf, 320);
            uint8_t *payload = new uint8_t[320];
            size_t payloadlen = encode_pcm((const int16_t *)speechIn, 160, payload);
            neteq_->InsertPacket(rtp_header, rtc::ArrayView<const uint8_t>(payload, payloadlen), kFirstReceiveTime);
            // InsertPacket(rtp_header, rtc::ArrayView<const uint8_t>(payload, 1920), kFirstReceiveTime);      
            kFirstTimestamp += 160;
            kFirstSequenceNumber++;
        }

        AudioFrame frame;
        bool muted = false;

        // GetAudioInternal(&frame, &muted);
        neteq_->GetAudio(&frame, &muted);
        if(frame.data()){
            fwrite(frame.data(), sizeof(int16_t), frame.samples_per_channel_ * frame.num_channels_, outfile);
        }
        frame.Reset();
        
        neteq_->GetAudio(&frame, &muted);
        // GetAudioInternal(&frame, &muted);
        if(frame.data()){
            fwrite(frame.data(), sizeof(int16_t), frame.samples_per_channel_ * frame.num_channels_, outfile);
        }
    }
    fclose(outfile);
    fclose(pcm);

    // RTPHeader rtp_header;
    // rtp_header.payloadType = kPayloadType;
    // rtp_header.sequenceNumber = kFirstSequenceNumber;
    // rtp_header.timestamp = kFirstTimestamp;

    // InsertPacket(rtp_header, payload, kFirstReceiveTime);
    
    // for (int i = 1; i < 10000; ++i){
    //     tick_timer_->Increment();

    //     if (i % 5 == 0)
    //     {
    //         rtp_header.timestamp += 480;
    //         rtp_header.sequenceNumber += 1;
    //         InsertPacket(rtp_header, payload, kFirstReceiveTime);
    //     }
    //     if (i % 3 == 0)
    //     {
    //         rtp_header.timestamp += 480;
    //         rtp_header.sequenceNumber += 1;
    //         InsertPacket(rtp_header, payload, kFirstReceiveTime);
    //     }
    // }
    // delay_manager_->ShowHistogram();
    // printf("target level:%d\n",delay_manager_->TargetLevel()>>8);

    return 0;
}
