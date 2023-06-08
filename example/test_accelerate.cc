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
#include "neteq/accelerate.h"
#include "neteq/defines.h"

static const int kInputSizeMs = 50;
static const int kOutputSizeMs = 30;
static const size_t kMaxFrameSize = 5760;  // 120 ms @ 48 kHz.
static const size_t kSyncBufferSize = kMaxFrameSize + 60 * 48;

#define kMode_Normal 0
#define kMode_Expand 1
#define kMode_Merge  2

using namespace webrtc;

int main(){

    int fs_hz = 48000;
    size_t channels = 1;

    std::unique_ptr<BackgroundNoise> background_noise_;
    std::unique_ptr<AudioMultiVector> algorithm_buffer_;
    std::unique_ptr<SyncBuffer> sync_buffer_;
    std::unique_ptr<Expand> expand_;
    std::unique_ptr<ExpandFactory> expand_factory_;
    std::unique_ptr<Merge> merge_;
    RandomVector random_vector_;
    std::unique_ptr<PostDecodeVad> vad_;

    std::unique_ptr<AccelerateFactory> accelerate_factory_;
    std::unique_ptr<Accelerate> accelerate_;

    int fs_mult_ = fs_hz / 8000;
    int samples_10ms = static_cast<size_t>(10 * 8 * fs_mult_);
    int input_size_samples_ = static_cast<size_t>(kInputSizeMs * 8 * fs_mult_);
    int output_size_samples_ = static_cast<size_t>(kOutputSizeMs * 8 * fs_mult_);
    int decoder_frame_length_ = 3 * output_size_samples_;  // Initialize to 30ms.

    // Reinit post-decode VAD with new sample rate.
    vad_.reset(new PostDecodeVad());
    vad_->Init();
    vad_->Enable();

    // Delete algorithm buffer and create a new one.
    algorithm_buffer_.reset(new AudioMultiVector(channels));

    // Delete sync buffer and create a new one.
    sync_buffer_.reset(new SyncBuffer(channels, kSyncBufferSize * fs_mult_));

    // Delete BackgroundNoise object and create a new one.
    background_noise_.reset(new BackgroundNoise(channels));

    // Reset random vector.
    random_vector_.Reset();

    expand_factory_.reset(new ExpandFactory);
    expand_.reset(expand_factory_->Create(background_noise_.get(),
    sync_buffer_.get(), &random_vector_,
    fs_hz, channels));

    accelerate_factory_.reset(new AccelerateFactory);
    accelerate_.reset(accelerate_factory_->Create(fs_hz, channels, *background_noise_));

    // Move index so that we create a small set of future samples (all 0).
    sync_buffer_->set_next_index(sync_buffer_->next_index() -
                    expand_->overlap_length());

    FILE *pcm = fopen("48.pcm", "rb");
    if(pcm == NULL) {
        printf("open pcm file failed!\n");
        return 0;
    }

    FILE *outfile = fopen("plc.pcm", "wb");
    AudioFrame frame;

    AudioMultiVector *mv = algorithm_buffer_.get();
    AudioVector &v = (*mv)[0];	
    int16_t buf[480 * 3];
    int16_t offset = 0;
    int read;
    int last_mode = kMode_Normal;
    int count = 0;

    while(!feof(pcm)){
        read = fread(buf + offset, sizeof(int16_t), samples_10ms, pcm);
        if(read != samples_10ms){
            printf("read: %d\n", read);
            break;
        }
        offset += samples_10ms;

        if (offset == samples_10ms * 3){
            offset = 0;
            size_t samples_removed;
            algorithm_buffer_->Clear();
            Accelerate::ReturnCodes return_code =
                accelerate_->Process(buf, samples_10ms * 3, false,
                            algorithm_buffer_.get(), &samples_removed);
                switch (return_code) {
                case Accelerate::kSuccess:
                    printf("accelerate success\n");
                    sync_buffer_->PushBack(*algorithm_buffer_);
                break;
                case Accelerate::kSuccessLowEnergy:
                    printf("accelerate low energy success\n");
                    sync_buffer_->PushBack(*algorithm_buffer_);
                break;
                case Accelerate::kNoStretch:
                    printf("accelerate no stretch\n");
                    sync_buffer_->PushBack(*algorithm_buffer_);
                break;
                case Accelerate::kError:
                    printf("accelerate error\n");
                break;
                }
        }

        if (sync_buffer_->FutureLength() > samples_10ms) {

            sync_buffer_->GetNextAudioInterleaved(samples_10ms, &frame);	
            if(frame.data()){
                fwrite(frame.data(), sizeof(int16_t), samples_10ms, outfile);
            }
        }
    }

    fflush(outfile);
    fclose(outfile);
    fclose(pcm);

    return 0;
}
