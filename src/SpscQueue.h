// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SpscQueue.h
//
// Lock-free single-producer / single-consumer ring buffer.
// Used to pass commands from the UI thread (producer) to the
// RT control loop thread (consumer) without mutexes.
//
// Constraints:
//   - Exactly ONE producer thread, ONE consumer thread.
//   - T must be trivially copyable.
//   - N must be a power of 2 (compile-time asserted).
//   - push() returns false if full (command dropped + caller logs).
//   - pop()  returns false if empty (no-op).
//
// Memory ordering:
//   - push(): write slot, then release-store m_writeIdx.
//   - pop():  acquire-load m_writeIdx, read slot, release-store m_readIdx.
// This prevents reordering of the slot write/read past the index update.
// ============================================================

#include <atomic>
#include <array>
#include <cstddef>

template<typename T, std::size_t N>
class SpscQueue
{
    static_assert((N & (N - 1)) == 0, "SpscQueue capacity N must be a power of 2");
    static_assert(N >= 2,             "SpscQueue capacity N must be at least 2");

public:
    // Called from producer (UI thread).
    // Returns true on success, false if the queue is full.
    bool push(const T& item)
    {
        const std::size_t w = m_writeIdx.load(std::memory_order_relaxed);
        const std::size_t wNext = (w + 1) & (N - 1);

        if (wNext == m_readIdx.load(std::memory_order_acquire))
            return false;  // full

        m_data[w] = item;
        m_writeIdx.store(wNext, std::memory_order_release);
        return true;
    }

    // Called from consumer (RT thread).
    // Returns true and fills `item` if a command was available, false if empty.
    bool pop(T& item)
    {
        const std::size_t r = m_readIdx.load(std::memory_order_relaxed);

        if (r == m_writeIdx.load(std::memory_order_acquire))
            return false;  // empty

        item = m_data[r];
        m_readIdx.store((r + 1) & (N - 1), std::memory_order_release);
        return true;
    }

    // Non-blocking check - safe to call from either thread (approximate).
    bool empty() const
    {
        return m_readIdx.load(std::memory_order_relaxed) ==
               m_writeIdx.load(std::memory_order_relaxed);
    }

private:
    std::array<T, N>         m_data{};
    std::atomic<std::size_t> m_writeIdx{0};  // advanced by producer after write
    std::atomic<std::size_t> m_readIdx{0};   // advanced by consumer after read

    // Pad to separate cache lines - prevents false sharing between
    // producer (writes m_writeIdx) and consumer (writes m_readIdx).
    static_assert(sizeof(std::atomic<std::size_t>) <= 64,
        "Unexpected atomic size -- cache line padding may be insufficient");
};
