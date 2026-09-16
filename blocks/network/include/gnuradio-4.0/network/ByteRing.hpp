#ifndef GNURADIO_NETWORK_BYTERING_HPP
#define GNURADIO_NETWORK_BYTERING_HPP

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <span>
#include <vector>

namespace gr::blocks::network {

/**
 * @brief A fixed-capacity byte queue over one contiguous allocation, dropping the oldest bytes on overrun.
 *
 * For a stream arriving in whole messages and leaving in whatever sizes a consumer asks for, where both ends move
 * bulk bytes and the queue itself should cost nothing but the copy. `std::deque<std::byte>` does the same job, but
 * its storage is a chain of 512-byte segments, so every insert and every read walks segment boundaries instead of
 * issuing a memcpy -- measured 39 % slower than this over a consumer's shapes (64 KiB in, 256 KiB out).
 *
 * Here the storage is one buffer and an offset pair: a read is at most two memcpys, a write is at most two, and
 * where nothing wraps it is one of each.
 *
 * Not thread-safe: serialization is the caller's, which typically already holds a lock for other reasons.
 */
class ByteRing {
    std::vector<std::byte> _buffer;
    std::size_t            _head = 0UZ; // index of the oldest byte
    std::size_t            _size = 0UZ; // bytes held

public:
    explicit ByteRing(std::size_t capacity = 0UZ) { reset(capacity); }

    // discards whatever is held
    void reset(std::size_t capacity) {
        _buffer.assign(capacity, std::byte{});
        _head = 0UZ;
        _size = 0UZ;
    }

    [[nodiscard]] std::size_t size() const noexcept { return _size; }
    [[nodiscard]] std::size_t capacity() const noexcept { return _buffer.size(); }
    [[nodiscard]] bool        empty() const noexcept { return _size == 0UZ; }

    void clear() noexcept {
        _head = 0UZ;
        _size = 0UZ;
    }

    // on overrun the newest bytes are the ones a live stream needs, so a push larger than the capacity keeps only
    // its own tail
    void push(std::span<const std::byte> src) noexcept {
        if (_buffer.empty() || src.empty()) {
            return;
        }
        if (src.size() >= _buffer.size()) {
            const std::size_t keep = _buffer.size();
            std::memcpy(_buffer.data(), src.data() + (src.size() - keep), keep);
            _head = 0UZ;
            _size = keep;
            return;
        }
        const std::size_t room = _buffer.size() - _size;
        if (src.size() > room) {
            dropOldest(src.size() - room);
        }
        const std::size_t tail  = (_head + _size) % _buffer.size();
        const std::size_t first = std::min(src.size(), _buffer.size() - tail);
        std::memcpy(_buffer.data() + tail, src.data(), first);
        if (first < src.size()) {
            std::memcpy(_buffer.data(), src.data() + first, src.size() - first);
        }
        _size += src.size();
    }

    // returns fewer than asked for only when the queue holds fewer
    std::size_t pop(std::byte* dst, std::size_t bytes) noexcept {
        const std::size_t take = peek(dst, bytes);
        dropOldest(take);
        return take;
    }

    // pop without dropping, for a caller that has to know it can commit before it does
    std::size_t peek(std::byte* dst, std::size_t bytes) const noexcept {
        if (_buffer.empty() || dst == nullptr) {
            return 0UZ;
        }
        const std::size_t take  = std::min(bytes, _size);
        const std::size_t first = std::min(take, _buffer.size() - _head);
        std::memcpy(dst, _buffer.data() + _head, first);
        if (first < take) {
            std::memcpy(dst + first, _buffer.data(), take - first);
        }
        return take;
    }

    // drops everything when fewer than `bytes` are held
    void dropOldest(std::size_t bytes) noexcept {
        if (_buffer.empty()) {
            return;
        }
        const std::size_t drop = std::min(bytes, _size);
        _head                  = (_head + drop) % _buffer.size();
        _size -= drop;
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_BYTERING_HPP
