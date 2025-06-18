#include <Bela.h>
#include <include/font.h>
#include <include/lsl_cpp.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>  // for memcpy
#include <fstream>
#include <include/linux_i2c.c>
#include <include/ssd1306.c>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// Configuration
#define USE_OLED_DISPLAY 1
const int kOledI2cDev = 1;

// Digital I/O pin configuration
const unsigned int eventStartPins[] = {0, 12};
const unsigned int eventEndPins[] = {1, 2};
const unsigned int kNumStartPins =
    sizeof(eventStartPins) / sizeof(eventStartPins[0]);
const unsigned int kNumEndPins = sizeof(eventEndPins) / sizeof(eventEndPins[0]);
const unsigned int kTotalPins = kNumStartPins + kNumEndPins;

// LSL Configuration
const char* streamPrefixFilter = "LSLTest";
float sampleTimeout = 0.0;
// Add this global variable at the top of your file
std::atomic<bool> pullSamplesRunning{false};

// Logging Configuration
uint64_t gElapsedFrames = 0;
const unsigned int fs = 44100;
const unsigned int periodSize = 16;
const size_t kEventBufferSize = 16384;  // Increased for better buffering
const size_t kFlushThreshold = 2048;
const int logRandomSuffix = rand() % 10000;

// Constants for pre-allocated buffers
const size_t kMaxStreamNameLength = 256;
const size_t kMaxStreamSourceLength = 256;
const size_t kMaxChannelsPerStream = 1;

// Lightweight event structure for real-time safe logging
struct RTTimingEvent {
  uint64_t system_frame;
  enum Type { PIN_CHANGE, LSL_SAMPLE } type;

  // Pin change data
  int pin_number;
  bool pin_state;
  bool is_start_pin;  // True if this is a start pin

  // LSL sample data (fixed-size for RT safety)
  char stream_name[kMaxStreamNameLength];
  char stream_source[kMaxStreamSourceLength];
  // static const size_t kMaxSampleStringLength = 256;
  uint64_t sample_data[kMaxChannelsPerStream];
  size_t sample_data_count;
  double sample_timestamp;  // This will store the interpolated LSL time
  double time_correction;   // Time correction offset for this inlet (LSL events only)

  // Pin states snapshot
  uint16_t pin_states_snapshot;  // Bit field for up to 16 pins

  RTTimingEvent()
      : system_frame(0),
        type(PIN_CHANGE),
        pin_number(0),
        pin_state(false),
        is_start_pin(false),
        sample_data_count(0),
        sample_timestamp(0.0),
        time_correction(0.0),
        pin_states_snapshot(0) {
    memset(stream_name, 0, kMaxStreamNameLength);
    memset(stream_source, 0, kMaxStreamSourceLength);
    memset(sample_data, 0, sizeof(sample_data));
  }
};

// Lock-free SPSC (Single Producer Single Consumer) queue for better RT
// performance
template <typename T, size_t Size>
class LockFreeSPSCQueue {
 private:
  alignas(64) std::atomic<size_t> write_pos{0};  // Cache line aligned
  alignas(64) std::atomic<size_t> read_pos{0};   // Cache line aligned
  T buffer[Size];

 public:
  bool push(const T& item) {
    size_t current_write = write_pos.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % Size;

    if (next_write == read_pos.load(std::memory_order_acquire)) {
      return false;  // Queue full
    }

    buffer[current_write] = item;
    write_pos.store(next_write, std::memory_order_release);
    return true;
  }

  bool pop(T& item) {
    size_t current_read = read_pos.load(std::memory_order_relaxed);

    if (current_read == write_pos.load(std::memory_order_acquire)) {
      return false;  // Queue empty
    }

    item = buffer[current_read];
    read_pos.store((current_read + 1) % Size, std::memory_order_release);
    return true;
  }

  size_t size_approx() const {
    size_t w = write_pos.load(std::memory_order_relaxed);
    size_t r = read_pos.load(std::memory_order_relaxed);
    return (w >= r) ? (w - r) : (Size - r + w);
  }
};

// Global variables
std::atomic<bool> shouldResolveStreams{true};
std::vector<lsl::stream_info> availableStreams;
std::vector<lsl::stream_inlet*> streamInlets;
std::vector<std::string> streamNames;
std::vector<std::string> streamSources;
std::vector<double> streamTimeCorrections;  // Store time corrections per inlet
bool streamsResolved = false;

// RT-safe event queue
LockFreeSPSCQueue<RTTimingEvent, kEventBufferSize> rtEventQueue;

// Separate queue for LSL samples (written by aux task, read by logger)
LockFreeSPSCQueue<RTTimingEvent, 4096> lslEventQueue;

// Pin state tracking - use atomics for thread safety
std::atomic<uint16_t> currentPinStatesAtomic{0};
uint16_t previousPinStates = 0;

// Timing variables for auxiliary task
std::atomic<double> lastLSLTimestamp{0.0};
std::atomic<bool> bufferFullFlag{false};

// Timestamp synchronization for interpolation
struct TimestampSync {
  uint64_t belaFrame;
  double lslTime;
  std::atomic<bool> valid{false};
};

// Use triple buffering for lock-free updates
TimestampSync syncBuffers[3];
std::atomic<int> currentSyncIndex{0};
std::atomic<int> previousSyncIndex{1};

// Auxiliary tasks
AuxiliaryTask gResolveStreamsTask;
AuxiliaryTask gPullSamplesTask;
AuxiliaryTask gLogWriterTask;
AuxiliaryTask gDisplayPinStatesTask;
AuxiliaryTask gTimestampUpdateTask;
AuxiliaryTask gTimeCorrectionUpdateTask;

// LSL resolver
lsl::continuous_resolver* resolver = nullptr;

// Function declarations
void resolveStreams(void*);
void pullSamples(void*);
void writeLogData(void*);
void displayPinStates(void*);
void updateLSLTimestamp(void*);
void updateTimeCorrections(void*);
uint16_t getPinStateBitfield();
void setPinStateBit(uint16_t& bitfield, int index, bool state);
bool getPinStateBit(uint16_t bitfield, int index);

// Helper functions
std::string generateLogFileName() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(1000, 9999);
  return std::string("./timing_log_") + std::to_string(dis(gen)) + ".csv";
}

const std::string kLogFileName = generateLogFileName();

// RT-safe pin state helpers
uint16_t getPinStateBitfield() {
  return currentPinStatesAtomic.load(std::memory_order_relaxed);
}

void setPinStateBit(uint16_t& bitfield, int index, bool state) {
  if (state) {
    bitfield |= (1 << index);
  } else {
    bitfield &= ~(1 << index);
  }
}

bool getPinStateBit(uint16_t bitfield, int index) {
  return (bitfield >> index) & 1;
}

// Update LSL timestamp with synchronization info
void updateLSLTimestamp(void*) {
  // Find the buffer that's not being used for reading
  int writeIndex = 2;  // Use the third buffer for writing
  int currentRead = currentSyncIndex.load(std::memory_order_acquire);
  int previousRead = previousSyncIndex.load(std::memory_order_acquire);

  // Make sure we're not writing to a buffer being read
  if (writeIndex == currentRead || writeIndex == previousRead) {
    writeIndex = (currentRead != 0 && previousRead != 0)   ? 0
                 : (currentRead != 1 && previousRead != 1) ? 1
                                                           : 2;
  }

  // Update the write buffer
  syncBuffers[writeIndex].belaFrame = gElapsedFrames;
  syncBuffers[writeIndex].lslTime = lsl::local_clock();
  syncBuffers[writeIndex].valid.store(true, std::memory_order_release);

  // Rotate buffers: current becomes previous, write buffer becomes current
  // int oldPrevious = previousSyncIndex.load(std::memory_order_relaxed);
  previousSyncIndex.store(currentRead, std::memory_order_release);
  currentSyncIndex.store(writeIndex, std::memory_order_release);

  // Also update simple timestamp for non-critical uses
  lastLSLTimestamp.store(syncBuffers[writeIndex].lslTime,
                         std::memory_order_relaxed);
}

// Update time corrections for all active inlets
void updateTimeCorrections(void*) {
  if (!streamsResolved || streamInlets.empty()) return;
  
  for (size_t i = 0; i < streamInlets.size(); i++) {
    if (streamInlets[i]) {
      try {
        // Get time correction - first call may take longer, subsequent calls are fast
        double correction = streamInlets[i]->time_correction(0.1);  // 100ms timeout
        
        // Ensure we have enough space in the corrections vector
        if (streamTimeCorrections.size() <= i) {
          streamTimeCorrections.resize(i + 1, 0.0);
        }
        
        streamTimeCorrections[i] = correction;
      } catch (std::exception& e) {
        rt_printf("Error getting time correction for stream %zu: %s\n", i, e.what());
      }
    }
  }
}

// RT-safe interpolation of LSL timestamp
double interpolateLSLTime(uint64_t currentFrame) {
  int currentIdx = currentSyncIndex.load(std::memory_order_acquire);
  int previousIdx = previousSyncIndex.load(std::memory_order_acquire);

  const TimestampSync& sync2 = syncBuffers[currentIdx];
  const TimestampSync& sync1 = syncBuffers[previousIdx];

  // Check if we have valid sync points
  if (!sync2.valid.load(std::memory_order_acquire) ||
      !sync1.valid.load(std::memory_order_acquire) ||
      sync2.belaFrame == sync1.belaFrame) {
    return sync2.lslTime;  // Can't interpolate, return latest
  }

  // Linear interpolation between sync points
  double frameSpan = static_cast<double>(sync2.belaFrame - sync1.belaFrame);
  double timeSpan = sync2.lslTime - sync1.lslTime;
  double framesSinceSync1 = static_cast<double>(currentFrame - sync1.belaFrame);

  // Clamp to prevent extrapolation beyond sync2
  if (framesSinceSync1 > frameSpan) {
    framesSinceSync1 = frameSpan;
  }

  return sync1.lslTime + (framesSinceSync1 * timeSpan / frameSpan);
}

bool setup(BelaContext* context, void* userData) {
  rt_printf("Using LSL library version: %d.%d\n", lsl::library_version() / 100,
            lsl::library_version() % 100);

  if (kNumStartPins != kNumEndPins) {
    rt_printf("Error: Mismatched number of event start and end pins.\n");
    return false;
  }

  rt_printf("Setting up event pins...\n");

  // Set up digital pins
  for (int i = 0; i < kNumStartPins; i++) {
    pinMode(context, 0, eventStartPins[i], INPUT);
    pinMode(context, 0, eventEndPins[i], INPUT);
  }

#ifdef USE_OLED_DISPLAY
  rt_printf("Setting up OLED display...\n");
  ssd1306_init(kOledI2cDev);
  ssd1306_oled_default_config(64, 128);
  ssd1306_oled_clear_screen();
  ssd1306_oled_set_XY(0, 0);
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"LSL Timing Test");
#endif

  // Create auxiliary tasks
  if ((gResolveStreamsTask = Bela_createAuxiliaryTask(&resolveStreams, 40,
                                                      "resolve-streams")) == 0)
    return false;

  if ((gPullSamplesTask =
           Bela_createAuxiliaryTask(&pullSamples, 80, "pull-samples")) == 0)
    return false;

  if ((gLogWriterTask =
           Bela_createAuxiliaryTask(&writeLogData, 50, "log-writer")) == 0)
    return false;

#if USE_OLED_DISPLAY
  if ((gDisplayPinStatesTask = Bela_createAuxiliaryTask(
           &displayPinStates, 30, "display-pin-states")) == 0)
    return false;
#endif

  if ((gTimestampUpdateTask = Bela_createAuxiliaryTask(
           &updateLSLTimestamp, 85, "timestamp-update")) == 0)
    return false;

  if ((gTimeCorrectionUpdateTask = Bela_createAuxiliaryTask(
           &updateTimeCorrections, 70, "time-correction-update")) == 0)
    return false;

  // Initialize log file
  std::ofstream logFile(kLogFileName, std::ios::out | std::ios::trunc);
  if (logFile.is_open()) {
    logFile << "system_timestamp,lsl_timestamp,event_type,pin_number,pin_state,is_start_pin,"
               "stream_name,stream_source,sample_timestamp,sample_data,time_correction,pin_states\n";
    logFile.close();
    rt_printf("Created log file: %s\n", kLogFileName.c_str());
  } else {
    rt_printf("Warning: Could not create log file\n");
  }

  // Create continuous resolver
  resolver = new lsl::continuous_resolver();

  // Initialize timestamp synchronization buffers
  double initialTime = lsl::local_clock();
  for (int i = 0; i < 3; i++) {
    syncBuffers[i].belaFrame = 0;
    syncBuffers[i].lslTime = initialTime;
    syncBuffers[i].valid.store(true, std::memory_order_release);
  }

  // Schedule initial tasks
  Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
  Bela_scheduleAuxiliaryTask(gDisplayPinStatesTask);
  Bela_scheduleAuxiliaryTask(gTimestampUpdateTask);

  rt_printf("Setup complete. Monitoring %d pins.\n", kTotalPins);
  rt_printf(
      "Bela context: %d audio channels, %d digital frames, period size: %d\n",
      context->audioInChannels, context->digitalFrames, periodSize);

  return true;
}

void render(BelaContext* context, void* userData) {
  gElapsedFrames = context->audioFramesElapsed;

  // Get current pin state bitfield
  uint16_t currentStates =
      currentPinStatesAtomic.load(std::memory_order_relaxed);
  uint16_t newStates = currentStates;

  // Get interpolated LSL timestamp for this render cycle
  double interpolatedLSLTime = interpolateLSLTime(gElapsedFrames);

  bool anyChange = false;

  // Process all digital frames
  for (unsigned int n = 0; n < context->digitalFrames; n++) {
    // Calculate precise frame timestamp
    uint64_t frameNumber = gElapsedFrames + n;
    double frameLSLTime =
        interpolatedLSLTime + (n * (1.0 / context->digitalSampleRate));

    // Check start pins
    for (unsigned int j = 0; j < kNumStartPins; j++) {
      bool state = digitalRead(context, n, eventStartPins[j]);
      bool prevState = getPinStateBit(previousPinStates, j);

      if (state != prevState) {
        setPinStateBit(newStates, j, state);
        setPinStateBit(previousPinStates, j, state);

        // Create RT-safe event with interpolated timestamp
        RTTimingEvent event;
        event.system_frame = frameNumber;
        event.type = RTTimingEvent::PIN_CHANGE;
        event.pin_number = eventStartPins[j];
        event.pin_state = state;
        event.is_start_pin = true;  // Mark as start pin
        event.pin_states_snapshot = newStates;
        event.sample_timestamp = frameLSLTime;  // Store interpolated LSL time

        if (!rtEventQueue.push(event)) {
          bufferFullFlag.store(true, std::memory_order_relaxed);
        }

        anyChange = true;
      }
    }

    // Check end pins
    for (unsigned int j = 0; j < kNumEndPins; j++) {
      bool state = digitalRead(context, n, eventEndPins[j]);
      int pinIndex = j + kNumStartPins;
      bool prevState = getPinStateBit(previousPinStates, pinIndex);

      if (state != prevState) {
        setPinStateBit(newStates, pinIndex, state);
        setPinStateBit(previousPinStates, pinIndex, state);

        // Create RT-safe event with interpolated timestamp
        RTTimingEvent event;
        event.system_frame = frameNumber;
        event.type = RTTimingEvent::PIN_CHANGE;
        event.pin_number = eventEndPins[j];
        event.pin_state = state;
        event.is_start_pin = false;  // Mark as end pin
        event.pin_states_snapshot = newStates;
        event.sample_timestamp = frameLSLTime;  // Store interpolated LSL time

        if (!rtEventQueue.push(event)) {
          bufferFullFlag.store(true, std::memory_order_relaxed);
        }

        anyChange = true;
      }
    }

    // Write silence to audio outputs
    for (unsigned int j = 0; j < context->audioOutChannels; j++) {
      audioWrite(context, n, j, 0.0f);
    }
  }

  // Update atomic pin states if changed
  if (anyChange) {
    currentPinStatesAtomic.store(newStates, std::memory_order_relaxed);
  }

  // Schedule auxiliary tasks with minimal overhead
  // renderCount increments once every render cycle
  // which is <period> frames
  static unsigned int renderCount = 0;

  renderCount++;

  // Update LSL timestamp synchronization every ~50ms for good interpolation
  if (renderCount % (fs / periodSize / 20) == 0) {
    Bela_scheduleAuxiliaryTask(gTimestampUpdateTask);
  }

  // Display update only when needed
#if USE_OLED_DISPLAY
  static unsigned int lastDisplayUpdate = 0;
  if (renderCount % (fs / periodSize / 10) == 0) {  // Update every 100ms
    Bela_scheduleAuxiliaryTask(gDisplayPinStatesTask);
    lastDisplayUpdate = renderCount;
  }
#endif

  // Stream resolution once per second
  if (renderCount % (fs / periodSize) == 0 && shouldResolveStreams) {
    Bela_scheduleAuxiliaryTask(gResolveStreamsTask);
  }
  
  // Update time corrections every 5 seconds when streams are available
  if (!streamInlets.empty() && renderCount % (fs / periodSize * 5) == 0) {
    Bela_scheduleAuxiliaryTask(gTimeCorrectionUpdateTask);
  }

  // Pull samples more frequently, when streams are available
  if (!streamInlets.empty() && renderCount % 10 == 0) {
    Bela_scheduleAuxiliaryTask(gPullSamplesTask);
  }

  // Schedule log writer based on queue size
  size_t queueSize = rtEventQueue.size_approx() + lslEventQueue.size_approx();
  if (queueSize >= kFlushThreshold ||
      (renderCount % 1000 == 0 && queueSize > 0)) {
    Bela_scheduleAuxiliaryTask(gLogWriterTask);
  }
}

void cleanup(BelaContext* context, void* userData) {
  rt_fprintf(stderr,
             "Cleaning up...Please wait for the logs to finish writing.\n");

  // Stop resolving streams first
  shouldResolveStreams = false;

  // Final flush of event buffers
  int flushAttempts = 0;
  while ((rtEventQueue.size_approx() > 0 || lslEventQueue.size_approx() > 0) &&
         flushAttempts < 100) {
    Bela_scheduleAuxiliaryTask(gLogWriterTask);
    usleep(10000);
    flushAttempts++;
  }

#ifdef USE_OLED_DISPLAY
  ssd1306_oled_clear_screen();
  ssd1306_oled_set_XY(0, 0);
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"Cleanup complete");
  ssd1306_end();
#endif

  // Clean up stream inlets properly
  for (auto inlet : streamInlets) {
    if (inlet) {
      try {
        inlet->close_stream();
      } catch (...) {
        // Ignore exceptions during cleanup
      }
      delete inlet;
    }
  }
  streamInlets.clear();
  streamNames.clear();
  streamSources.clear();
  streamTimeCorrections.clear();

  // Clean up resolver
  if (resolver) {
    delete resolver;
    resolver = nullptr;
  }

  rt_printf("Cleanup complete. Events remaining: RT=%zu, LSL=%zu\n",
            rtEventQueue.size_approx(), lslEventQueue.size_approx());
}

// Auxiliary task implementations

void resolveStreams(void*) {
  bool needToReopen = streamInlets.empty();
  availableStreams = resolver->results();

  if (availableStreams.empty()) {
#ifdef USE_OLED_DISPLAY
    ssd1306_oled_clear_line(2);
    ssd1306_oled_set_XY(0, 2);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"No streams");
#endif
    return;
  }

  if (needToReopen) {
    // Clean up existing inlets
    for (auto inlet : streamInlets) {
      if (inlet) {
        inlet->close_stream();
        delete inlet;
      }
    }
    streamInlets.clear();
    streamNames.clear();
    streamSources.clear();
    streamTimeCorrections.clear();

    size_t validStreams = 0;
    for (const auto& info : availableStreams) {
      // Check if stream name contains the filter prefix
      if (!info.name().empty() &&
          info.name().find(streamPrefixFilter) != std::string::npos) {
        if (info.channel_format() != lsl::cf_int64) {
          rt_printf("Warning: Stream %s has format %d, expected int64 (%d)\n",
                    info.name().c_str(),
                    static_cast<int>(info.channel_format()),
                    static_cast<int>(lsl::cf_int64));
          // You can choose to skip this stream or handle format conversion
          continue;
        }
        try {
          lsl::stream_inlet* inlet = new lsl::stream_inlet(info, 10, 1, true);
          streamInlets.push_back(inlet);
          streamNames.push_back(info.name());
          streamSources.push_back(info.source_id());
          inlet->open_stream(1.0);
          
          // Get initial time correction (may take longer first time)
          try {
            double initial_correction = inlet->time_correction(2.0);  // 2s timeout for initial
            streamTimeCorrections.push_back(initial_correction);
          } catch (std::exception& e) {
            rt_printf("Warning: Could not get initial time correction for %s: %s\n", 
                     info.name().c_str(), e.what());
            streamTimeCorrections.push_back(0.0);  // Default to 0 if failed
          }
          
          validStreams++;

          rt_printf("Opened stream: %s (%s), %d channels, format: %d\n",
                    info.name().c_str(), info.type().c_str(),
                    info.channel_count(),
                    static_cast<int>(info.channel_format()));
        } catch (std::exception& e) {
          rt_printf("Error creating inlet for %s: %s\n", info.name().c_str(),
                    e.what());
        }
      }
    }

#ifdef USE_OLED_DISPLAY
    ssd1306_oled_clear_line(2);
    ssd1306_oled_set_XY(0, 2);
    std::string streamStr = "Streams: " + std::to_string(validStreams) + " / " +
                            std::to_string(availableStreams.size());
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)streamStr.c_str());
#endif

    streamsResolved = (validStreams > 0);
  }
}

void displayPinStates(void*) {
#ifdef USE_OLED_DISPLAY
  uint16_t states = currentPinStatesAtomic.load(std::memory_order_relaxed);

  ssd1306_oled_clear_line(4);
  ssd1306_oled_set_XY(0, 4);
  std::string pinStateStr = "P: |";
  for (int i = 0; i < kTotalPins; i++) {
    pinStateStr += (getPinStateBit(states, i) ? "X|" : "-|");
  }
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)pinStateStr.c_str());

  if (bufferFullFlag.load(std::memory_order_relaxed)) {
    ssd1306_oled_clear_line(5);
    ssd1306_oled_set_XY(0, 5);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"E: BUF FULL!!!");
  }
#endif
}

void pullSamples(void*) {
  if (!streamsResolved || streamInlets.empty()) return;

  for (size_t i = 0; i < streamInlets.size(); i++) {
    if (!streamInlets[i]) {
      continue;
    }

    try {
      // Create a proper buffer for int64 data
      std::vector<int64_t> buffer(kMaxChannelsPerStream);

      double sampleTimestamp =
          streamInlets[i]->pull_sample(buffer, sampleTimeout);

      if (sampleTimestamp != 0.0) {
        // Create RT-safe event for LSL sample
        RTTimingEvent event;
        event.system_frame = gElapsedFrames;
        event.type = RTTimingEvent::LSL_SAMPLE;

        // Copy stream name safely
        strncpy(event.stream_name, streamNames[i].c_str(),
                kMaxStreamNameLength - 1);
        event.stream_name[kMaxStreamNameLength - 1] = '\0';

        // Copy stream source safely
        strncpy(event.stream_source, streamSources[i].c_str(),
                kMaxStreamSourceLength - 1);
        event.stream_source[kMaxStreamSourceLength - 1] = '\0';

        // Copy sample data
        event.sample_data_count =
            std::min(static_cast<size_t>(kMaxChannelsPerStream), buffer.size());
        for (size_t j = 0; j < event.sample_data_count; j++) {
          event.sample_data[j] = static_cast<uint64_t>(buffer[j]);
        }

        event.sample_timestamp = sampleTimestamp;
        
        // Add time correction if available for this inlet
        if (i < streamTimeCorrections.size()) {
          event.time_correction = streamTimeCorrections[i];
        } else {
          event.time_correction = 0.0;
        }
        
        event.pin_states_snapshot =
            currentPinStatesAtomic.load(std::memory_order_relaxed);

        // Push to LSL queue
        if (!lslEventQueue.push(event)) {
          rt_printf("LSL event queue full\n");
        }
      }
    } catch (lsl::lost_error& e) {
      rt_printf("Stream %s lost: %s\n", streamNames[i].c_str(), e.what());
      streamInlets[i]->close_stream();
      delete streamInlets[i];
      streamInlets[i] = nullptr;
      shouldResolveStreams = true;
    } catch (std::exception& e) {
      rt_printf("Error pulling sample from %s: %s\n", streamNames[i].c_str(),
                e.what());
    } catch (...) {
      rt_printf("Unknown error pulling sample from %s\n",
                streamNames[i].c_str());
    }
  }

  // Clean up null pointers
  streamInlets.erase(
      std::remove_if(streamInlets.begin(), streamInlets.end(),
                     [](lsl::stream_inlet* inlet) { return inlet == nullptr; }),
      streamInlets.end());

  // Keep streamNames in sync
  if (streamInlets.size() != streamNames.size()) {
    streamNames.resize(streamInlets.size());
  }

  // Update stream sources
  if (streamSources.size() != streamInlets.size()) {
    streamSources.resize(streamInlets.size());
  }
  
  // Update time corrections
  if (streamTimeCorrections.size() != streamInlets.size()) {
    streamTimeCorrections.resize(streamInlets.size(), 0.0);
  }

  if (streamInlets.empty()) {
    streamsResolved = false;
  }
}

void writeLogData(void*) {
  std::ofstream logFile(kLogFileName, std::ios::out | std::ios::app);
  if (!logFile.is_open()) {
    rt_printf("Error: Could not open log file for writing\n");
    return;
  }

  RTTimingEvent event;
  size_t eventsWritten = 0;

  // Process RT events first (higher priority)
  while (rtEventQueue.pop(event) && eventsWritten < 1000) {
    logFile << event.system_frame << "," << std::fixed << std::setprecision(9)
            << event.sample_timestamp << ",";

    if (event.type == RTTimingEvent::PIN_CHANGE) {
      logFile << "PIN_CHANGE," << event.pin_number << ","
              << (event.pin_state ? "1" : "0") << ","
              << (event.is_start_pin ? "1" : "0") << ",,,,,,";
    } else {
      logFile << "LSL_SAMPLE,,,," << event.stream_name << ","
              << event.stream_source << ","
              << event.sample_timestamp << ",";

      // Output string data from uint64_t sample_data
      for (size_t i = 0; i < event.sample_data_count; i++) {
        logFile << event.sample_data[i];
        if (i < event.sample_data_count - 1) logFile << ";";
      }
      logFile << "," << std::fixed << std::setprecision(9) << event.time_correction << ",";
    }

    // Add pin states
    for (int i = 0; i < kTotalPins; i++) {
      logFile << (getPinStateBit(event.pin_states_snapshot, i) ? "1" : "0");
      if (i < kTotalPins - 1) logFile << ";";
    }

    logFile << "\n";
    eventsWritten++;
  }

  // Process LSL events if we have room
  while (lslEventQueue.pop(event) && eventsWritten < 1500) {
    double currentLSLTime = lastLSLTimestamp.load(std::memory_order_relaxed);

    logFile << event.system_frame << "," << std::fixed << std::setprecision(9)
            << currentLSLTime << ","
            << "LSL_SAMPLE,,,," << event.stream_name << ","
            << event.stream_source << ","
            << event.sample_timestamp << ",";

    for (size_t i = 0; i < event.sample_data_count; i++) {
      logFile << event.sample_data[i];
      if (i < event.sample_data_count - 1) logFile << ";";
    }
    logFile << "," << std::fixed << std::setprecision(9) << event.time_correction << ",";

    for (int i = 0; i < kTotalPins; i++) {
      logFile << (getPinStateBit(event.pin_states_snapshot, i) ? "1" : "0");
      if (i < kTotalPins - 1) logFile << ";";
    }

    logFile << "\n";
    eventsWritten++;
  }

  logFile.close();

  if (eventsWritten > 0) {
    rt_printf("Wrote %zu events. Queues: RT=%zu, LSL=%zu\n", eventsWritten,
              rtEventQueue.size_approx(), lslEventQueue.size_approx());
  }
}
