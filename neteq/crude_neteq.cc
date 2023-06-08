#include "neteq/crude_neteq.h"

#include <memory>

#include "neteq/crude_neteq_impl.h"

namespace webrtc {

CrudeNetEq::Config::Config() = default;
CrudeNetEq::Config::Config(const Config&) = default;
CrudeNetEq::Config::Config(Config&&) = default;
CrudeNetEq::Config::~Config() = default;
CrudeNetEq::Config& CrudeNetEq::Config::operator=(const Config&) = default;
CrudeNetEq::Config& CrudeNetEq::Config::operator=(Config&&) = default;

std::string CrudeNetEq::Config::ToString() const {
//   char buf[1024];
//   rtc::SimpleStringBuilder ss(buf);
//   ss << "sample_rate_hz=" << sample_rate_hz << ", enable_post_decode_vad="
//      << (enable_post_decode_vad ? "true" : "false")
//      << ", max_packets_in_buffer=" << max_packets_in_buffer
//      << ", enable_fast_accelerate="
//      << (enable_fast_accelerate ? " true" : "false")
//      << ", enable_muted_state=" << (enable_muted_state ? " true" : "false");
//   return ss.str();
    return std::string("");
}

CrudeNetEq* CrudeNetEq::Create(const CrudeNetEq::Config& config, std::unique_ptr<AudioDecoder> &decoder) {
    return new CrudeNetEqImpl(config, decoder);
}

}  // namespace webrtc
