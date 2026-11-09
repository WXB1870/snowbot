// Test-process library for Orin Humble / Fast DDS 2.6.12. Never installed by colcon.
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <mutex>
#include <time.h>

#include <fastdds/rtps/reader/RTPSReader.h>
#include <rcl/subscription.h>
#include <rmw_fastrtps_cpp/get_subscriber.hpp>

using namespace eprosima::fastrtps::rtps;

struct Enqueue {
    uint64_t boot, sample, header_ns, begin_ns, end_ns;
};

namespace {
GUID_t target;
std::atomic<bool> bound{false};
std::array<Enqueue, 256> ring{};
std::mutex lock;
size_t written = 0;
std::atomic<uint64_t> lost{0};

uint64_t now_ns() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}

template <typename T> T load(const uint8_t* p) {
    T result;
    std::memcpy(&result, p, sizeof(result));
    return result;
}

size_t align_up(size_t n, size_t alignment) {
    return (n + alignment - 1) & ~(alignment - 1);
}

bool observation_prefix(const CacheChange_t& change, Enqueue& record) {
    const auto& payload = change.serializedPayload;
    const auto* p = payload.data;
    if (!p || payload.length < 16 || p[0] != 0 || p[1] != 1) return false;
    // Little-endian XCDR1. Alignment is relative to the end of the encapsulation.
    p += 4;
    const size_t frame_len = load<uint32_t>(p + 8);
    const size_t wrench = align_up(12 + frame_len, 4);
    const size_t data_no = align_up(wrench + 12 * sizeof(float) + 1, 2);
    const size_t ages = align_up(data_no + 2 * sizeof(uint16_t), 8);
    const size_t boot = ages + 2 * sizeof(double);
    if (boot + 16 > payload.length - 4) return false;
    record.boot = load<uint64_t>(p + boot);
    record.sample = load<uint64_t>(p + boot + 8);
    record.header_ns = uint64_t(load<uint32_t>(p)) * 1000000000 + load<uint32_t>(p + 4);
    return true;
}
}  // namespace

extern "C" bool bench_bind_reader(rcl_subscription_t* sub) {
    if (bound.load()) return false;
    auto* reader = rmw_fastrtps_cpp::get_datareader(rcl_subscription_get_rmw_handle(sub));
    if (!reader) return false;
    target = reader->guid();
    bound.store(true, std::memory_order_release);
    return true;
}

bool ReaderHistory::add_change(CacheChange_t* change) {
    using Add = bool (*)(ReaderHistory*, CacheChange_t*);
    static const auto add = reinterpret_cast<Add>(dlsym(RTLD_NEXT,
        "_ZN8eprosima8fastrtps4rtps13ReaderHistory10add_changeEPNS1_13CacheChange_tE"));
    if (!add) std::abort();
    if (!bound.load(std::memory_order_acquire) || !mp_reader || mp_reader->getGuid() != target)
        return add(this, change);
    Enqueue record{};
    const bool decoded = observation_prefix(*change, record);
    record.begin_ns = now_ns();
    const bool added = add(this, change);
    record.end_ns = now_ns();
    if (added) {
        std::unique_lock<std::mutex> guard(lock, std::try_to_lock);
        if (decoded && guard.owns_lock()) ring[written++ % ring.size()] = record;
        else ++lost;
    }
    return added;
}

extern "C" size_t bench_snapshot(Enqueue* dst, size_t capacity, uint64_t* dropped) {
    std::lock_guard<std::mutex> guard(lock);
    const size_t n = std::min(capacity, std::min(written, ring.size()));
    for (size_t i = 0; i < n; ++i) dst[i] = ring[(written - n + i) % ring.size()];
    *dropped = lost.load() + written - n;
    return n;
}
