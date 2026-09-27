// Fixed-capacity FIFO that numbers every entry, so readers can page with since(seq).
#pragma once
#include <cstddef>
#include <cstdint>

template <typename T, size_t N>
class SeqRing {
 public:
  struct Item {
    uint32_t seq = 0;
    T value;
  };

  uint32_t push(const T &value) {
    items_[head_].seq = ++lastSeq_;
    items_[head_].value = value;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
    return lastSeq_;
  }

  // Copies up to max entries with seq > afterSeq, oldest first; returns how many.
  size_t since(uint32_t afterSeq, Item *out, size_t max) const {
    size_t written = 0;
    size_t oldest = (head_ + N - count_) % N;
    for (size_t i = 0; i < count_ && written < max; i++) {
      const Item &item = items_[(oldest + i) % N];
      if (item.seq > afterSeq) out[written++] = item;
    }
    return written;
  }

  uint32_t lastSeq() const { return lastSeq_; }
  size_t size() const { return count_; }

 private:
  Item items_[N];
  size_t head_ = 0;
  size_t count_ = 0;
  uint32_t lastSeq_ = 0;
};
