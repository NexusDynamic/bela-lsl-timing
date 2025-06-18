#include <Bela.h>
#include <include/font.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
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
const unsigned int kNumStartPins = sizeof(eventStartPins) / sizeof(eventStartPins[0]);
const unsigned int kNumEndPins = sizeof(eventEndPins) / sizeof(eventEndPins[0]);
const unsigned int kTotalPins = kNumStartPins + kNumEndPins;

// Trigger state configuration
const bool eventStartPinTriggerState = true;  // HIGH
const bool eventEndPinTriggerState = false;  // LOW

// Monitor pin combo configuration (pin 0 -> pin 1)
const unsigned int monitorStartPin = 0;
const unsigned int monitorEndPin = 1;

// Logging Configuration
uint64_t gElapsedFrames = 0;
const unsigned int fs = 44100;
const unsigned int periodSize = 16;
const size_t kEventBufferSize = 16384;
const size_t kFlushThreshold = 2048;

// Lightweight event structure for real-time safe logging
struct RTTimingEvent {
  uint64_t frame_number;
  int pin_number;
  bool pin_state;
  bool is_start_pin;
  bool trigger_active;
  uint16_t pin_states_snapshot;

  RTTimingEvent()
      : frame_number(0),
        pin_number(0),
        pin_state(false),
        is_start_pin(false),
        trigger_active(false),
        pin_states_snapshot(0) {}
};

// Lock-free SPSC queue for better RT performance
template <typename T, size_t Size>
class LockFreeSPSCQueue {
 private:
  alignas(64) std::atomic<size_t> write_pos{0};
  alignas(64) std::atomic<size_t> read_pos{0};
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
LockFreeSPSCQueue<RTTimingEvent, kEventBufferSize> rtEventQueue;

// Pin state tracking
std::atomic<uint16_t> currentPinStatesAtomic{0};
uint16_t previousPinStates = 0;

// Latency monitoring
struct LatencyStats {
  std::atomic<double> totalLatencyFrames{0.0};
  std::atomic<uint64_t> eventCount{0};
  std::atomic<uint64_t> lastStartFrame{0};
  std::atomic<bool> waitingForEnd{false};
};

LatencyStats latencyStats;

// Auxiliary tasks
AuxiliaryTask gLogWriterTask;
AuxiliaryTask gDisplayTask;

// Function declarations
void writeLogData(void*);
void displayInfo(void*);
uint16_t getPinStateBitfield();
void setPinStateBit(uint16_t& bitfield, int index, bool state);
bool getPinStateBit(uint16_t bitfield, int index);
bool isTriggerActive(bool pinState, bool isStartPin);

// Helper functions
std::string generateLogFileName() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(1000, 9999);
  return std::string("./timing_log_simplified_") + std::to_string(dis(gen)) + ".csv";
}

const std::string kLogFileName = generateLogFileName();

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

bool isTriggerActive(bool pinState, bool isStartPin) {
  if (isStartPin) {
    return pinState == eventStartPinTriggerState;
  } else {
    return pinState == eventEndPinTriggerState;
  }
}


bool setup(BelaContext* context, void* userData) {
  if (kNumStartPins != kNumEndPins) {
    rt_printf("Error: Mismatched number of event start and end pins.\n");
    return false;
  }

  rt_printf("Setting up simplified timing measurement...\n");

  // Set up digital pins
  for (int i = 0; i < kNumStartPins; i++) {
    pinMode(context, 0, eventStartPins[i], INPUT);
    pinMode(context, 0, eventEndPins[i], INPUT);
  }

#if USE_OLED_DISPLAY
  rt_printf("Setting up OLED display...\n");
  ssd1306_init(kOledI2cDev);
  ssd1306_oled_default_config(64, 128);
  ssd1306_oled_clear_screen();
  ssd1306_oled_set_XY(0, 0);
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"Timing Simplified");
#endif

  // Create auxiliary tasks
  if ((gLogWriterTask = Bela_createAuxiliaryTask(&writeLogData, 50, "log-writer")) == 0)
    return false;

#if USE_OLED_DISPLAY
  if ((gDisplayTask = Bela_createAuxiliaryTask(&displayInfo, 30, "display-info")) == 0)
    return false;
#endif

  // Initialize log file
  std::ofstream logFile(kLogFileName, std::ios::out | std::ios::trunc);
  if (logFile.is_open()) {
    logFile << "frame_number,pin_number,pin_state,is_start_pin,trigger_active,pin_states\n";
    logFile.close();
    rt_printf("Created log file: %s\n", kLogFileName.c_str());
  } else {
    rt_printf("Warning: Could not create log file\n");
  }

  rt_printf("Setup complete. Monitoring %d pins.\n", kTotalPins);
  rt_printf("Start pin trigger state: %s, End pin trigger state: %s\n",
           eventStartPinTriggerState ? "HIGH" : "LOW",
           eventEndPinTriggerState ? "HIGH" : "LOW");
  rt_printf("Monitor combo: Pin %d -> Pin %d\n", monitorStartPin, monitorEndPin);

  return true;
}

void render(BelaContext* context, void* userData) {
  gElapsedFrames = context->audioFramesElapsed;

  // Get current pin state bitfield
  uint16_t currentStates = currentPinStatesAtomic.load(std::memory_order_relaxed);
  uint16_t newStates = currentStates;

  bool anyChange = false;

  // Process all digital frames
  for (unsigned int n = 0; n < context->digitalFrames; n++) {
    uint64_t frameNumber = gElapsedFrames + n;

    // Check start pins
    for (unsigned int j = 0; j < kNumStartPins; j++) {
      bool state = digitalRead(context, n, eventStartPins[j]);
      bool prevState = getPinStateBit(previousPinStates, j);

      if (state != prevState) {
        setPinStateBit(newStates, j, state);
        setPinStateBit(previousPinStates, j, state);

        bool triggerActive = isTriggerActive(state, true);

        // Monitor pin combo logic
        if (eventStartPins[j] == monitorStartPin && triggerActive) {
          latencyStats.lastStartFrame.store(frameNumber, std::memory_order_relaxed);
          latencyStats.waitingForEnd.store(true, std::memory_order_relaxed);
        }

        RTTimingEvent event;
        event.frame_number = frameNumber;
        event.pin_number = eventStartPins[j];
        event.pin_state = state;
        event.is_start_pin = true;
        event.trigger_active = triggerActive;
        event.pin_states_snapshot = newStates;

        rtEventQueue.push(event);
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

        bool triggerActive = isTriggerActive(state, false);

        // Monitor pin combo logic
        if (eventEndPins[j] == monitorEndPin && triggerActive && 
            latencyStats.waitingForEnd.load(std::memory_order_relaxed)) {
          uint64_t startFrame = latencyStats.lastStartFrame.load(std::memory_order_relaxed);
          double latencyFrames = static_cast<double>(frameNumber - startFrame);
          
          uint64_t currentCount = latencyStats.eventCount.load(std::memory_order_relaxed);
          double currentTotal = latencyStats.totalLatencyFrames.load(std::memory_order_relaxed);
          
          latencyStats.totalLatencyFrames.store(currentTotal + latencyFrames, std::memory_order_relaxed);
          latencyStats.eventCount.store(currentCount + 1, std::memory_order_relaxed);
          latencyStats.waitingForEnd.store(false, std::memory_order_relaxed);
        }

        RTTimingEvent event;
        event.frame_number = frameNumber;
        event.pin_number = eventEndPins[j];
        event.pin_state = state;
        event.is_start_pin = false;
        event.trigger_active = triggerActive;
        event.pin_states_snapshot = newStates;

        rtEventQueue.push(event);
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

  // Schedule auxiliary tasks
  static unsigned int renderCount = 0;
  renderCount++;

#if USE_OLED_DISPLAY
  if (renderCount % (fs / periodSize / 10) == 0) {
    Bela_scheduleAuxiliaryTask(gDisplayTask);
  }
#endif

  // Schedule log writer based on queue size
  size_t queueSize = rtEventQueue.size_approx();
  if (queueSize >= kFlushThreshold || (renderCount % 1000 == 0 && queueSize > 0)) {
    Bela_scheduleAuxiliaryTask(gLogWriterTask);
  }
}

void cleanup(BelaContext* context, void* userData) {
  rt_printf("Cleaning up...\n");

  // Final flush of event buffers
  int flushAttempts = 0;
  while (rtEventQueue.size_approx() > 0 && flushAttempts < 100) {
    Bela_scheduleAuxiliaryTask(gLogWriterTask);
    usleep(10000);
    flushAttempts++;
  }

#if USE_OLED_DISPLAY
  ssd1306_oled_clear_screen();
  ssd1306_oled_set_XY(0, 0);
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"Cleanup complete");
  ssd1306_end();
#endif

  rt_printf("Cleanup complete. Events remaining: %zu\n", rtEventQueue.size_approx());
}

void displayInfo(void*) {
#if USE_OLED_DISPLAY
  uint16_t states = currentPinStatesAtomic.load(std::memory_order_relaxed);

  // Display pin states
  ssd1306_oled_clear_line(2);
  ssd1306_oled_set_XY(0, 2);
  std::string pinStateStr = "P: |";
  for (int i = 0; i < kTotalPins; i++) {
    pinStateStr += (getPinStateBit(states, i) ? "X|" : "-|");
  }
  ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)pinStateStr.c_str());

  // Display latency stats
  uint64_t eventCount = latencyStats.eventCount.load(std::memory_order_relaxed);
  if (eventCount > 0) {
    double totalLatencyFrames = latencyStats.totalLatencyFrames.load(std::memory_order_relaxed);
    double avgLatencyFrames = totalLatencyFrames / eventCount;
    // Convert frames to milliseconds (assuming 44.1kHz sample rate)
    double avgLatencyMs = (avgLatencyFrames / 44100.0) * 1000.0;
    
    ssd1306_oled_clear_line(3);
    ssd1306_oled_set_XY(0, 3);
    std::string latencyStr = "Avg: " + std::to_string((int)avgLatencyMs) + "ms";
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)latencyStr.c_str());
    
    ssd1306_oled_clear_line(4);
    ssd1306_oled_set_XY(0, 4);
    std::string countStr = "Cnt: " + std::to_string(eventCount);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)countStr.c_str());
  }
#endif
}

void writeLogData(void*) {
  std::ofstream logFile(kLogFileName, std::ios::out | std::ios::app);
  if (!logFile.is_open()) {
    rt_printf("Error: Could not open log file for writing\n");
    return;
  }

  RTTimingEvent event;
  size_t eventsWritten = 0;

  while (rtEventQueue.pop(event) && eventsWritten < 1000) {
    logFile << event.frame_number << ","
            << event.pin_number << ","
            << (event.pin_state ? "1" : "0") << ","
            << (event.is_start_pin ? "1" : "0") << ","
            << (event.trigger_active ? "1" : "0") << ",";

    // Add pin states
    for (int i = 0; i < kTotalPins; i++) {
      logFile << (getPinStateBit(event.pin_states_snapshot, i) ? "1" : "0");
      if (i < kTotalPins - 1) logFile << ";";
    }

    logFile << "\n";
    eventsWritten++;
  }

  logFile.close();

  if (eventsWritten > 0) {
    rt_printf("Wrote %zu events. Queue: %zu\n", eventsWritten, rtEventQueue.size_approx());
  }
}
