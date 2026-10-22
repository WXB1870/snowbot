#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace position_session {
// One writer: servo admission commit. One reader: non-periodic ROS timer.
// No ROS, allocation, locks, or ownership changes in the writer.
struct AdmissionReceipt {
 uint64_t session=0,sequence=0,boot=0,sample=0;
 int64_t decided_ros=0;
 bool accepted=false;
 std::array<char,160> reason{};
};
class ReceiptQueue {
 static constexpr unsigned capacity=8;
 std::array<AdmissionReceipt,capacity> entries_{};
 std::atomic<unsigned> head_{0},tail_{0};
 std::atomic<uint64_t> dropped_{0};
 static_assert(std::atomic<unsigned>::is_always_lock_free && std::atomic<uint64_t>::is_always_lock_free,"receipt atomics must be lock free");
public:
 bool push(uint64_t session,uint64_t sequence,uint64_t boot,uint64_t sample,
           int64_t ros,bool accepted,const char* reason)noexcept {
  const auto head=head_.load(std::memory_order_relaxed),next=(head+1)%capacity;
  if(next==tail_.load(std::memory_order_acquire)){dropped_.fetch_add(1,std::memory_order_relaxed);return false;}
  auto& entry=entries_[head];entry.session=session;entry.sequence=sequence;
  entry.boot=boot;entry.sample=sample;entry.decided_ros=ros;entry.accepted=accepted;
  size_t i=0;if(reason)for(;i+1<entry.reason.size() && reason[i];++i)entry.reason[i]=reason[i];
  entry.reason[i]='\0';head_.store(next,std::memory_order_release);return true;
 }
 bool pop(AdmissionReceipt& result)noexcept {
  const auto tail=tail_.load(std::memory_order_relaxed);
  if(tail==head_.load(std::memory_order_acquire))return false;
  result=entries_[tail];tail_.store((tail+1)%capacity,std::memory_order_release);return true;
 }
 uint64_t dropped()const noexcept{return dropped_.load(std::memory_order_relaxed);}
};
} // namespace position_session
