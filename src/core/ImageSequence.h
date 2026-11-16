// Image sequences for playback (the Sequence Player, like Houdini's MPlay).
//
// findSequence() turns any one frame of a numbered sequence (shot.0012.png) into the whole set of
// frames next to it. SequenceCache decodes frames to 8-bit RGBA on background threads, ahead of the
// playhead in the playback direction, keeping at most a memory budget of frames (the ones nearest
// ahead of the playhead win).
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hd {

struct Image;

struct SequenceInfo {
    std::string label;              // e.g. "shot.####.png"
    std::vector<int> frames;        // frame numbers, ascending
    std::vector<std::string> files; // one path per frame
    bool linear = false;            // EXR / HDR: linear values, shown with exposure and sRGB
    size_t size() const { return files.size(); }
};

// Every file next to `anyFrame` with the same name apart from its frame number (the last group of
// digits before the extension). A file without a number is a one-frame sequence.
bool findSequence(const std::string& anyFrame, SequenceInfo& out, std::string& error);

class SequenceCache {
public:
    struct Frame {
        int w = 0, h = 0;
        std::vector<uint8_t> rgba; // top row first
        std::string error;         // failed to load (then w = h = 0)
    };

    explicit SequenceCache(int threads = 0); // 0: a few, by core count
    ~SequenceCache();
    SequenceCache(const SequenceCache&) = delete;
    SequenceCache& operator=(const SequenceCache&) = delete;

    void open(const SequenceInfo& seq, size_t budgetBytes, float exposure = 0);
    void close();
    const SequenceInfo& sequence() const { return seq_; }

    // Where playback is and which way it goes (+1 / -1): loading follows it.
    void setPlayhead(int index, int direction);
    // Exposure in stops for linear sequences (reloads them).
    void setExposure(float stops);
    float exposure() const { return exposure_; }

    // The decoded frame, or null while it is not loaded yet.
    std::shared_ptr<const Frame> frame(int index) const;
    bool loaded(int index) const;
    std::vector<char> loadedMask() const;
    size_t bytes() const;
    size_t budget() const { return budget_; }
    // Waits until frame `index` is loaded (tests, exporting). False on timeout.
    bool waitFor(int index, int timeoutMs);

    // The 8-bit display conversion used for a frame (exposed for tests).
    static void toDisplay(const Image& img, bool linear, float exposure, Frame& out);

private:
    void worker();
    int pickLocked() const; // next index to load, -1 none
    int rankLocked(int index) const;

    mutable std::mutex m_;
    std::condition_variable cv_, loadedCv_;
    std::vector<std::thread> threads_;
    bool quit_ = false;
    uint64_t gen_ = 0;
    SequenceInfo seq_;
    std::vector<std::shared_ptr<const Frame>> frames_;
    std::vector<char> inFlight_;
    size_t bytes_ = 0, budget_ = 0, frameBytes_ = 0;
    int playhead_ = 0, dir_ = 1;
    float exposure_ = 0;
};

} // namespace hd
