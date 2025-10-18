# Asynchronous NeoPixel Animation System

## Overview

This document describes the asynchronous NeoPixel animation system implemented for the Mercator Origins Lantern IO project. The system allows LED animations to run without blocking the main program loop, enabling responsive sensor reading, serial communication, and other critical operations.

## Problem Solved

The original code used blocking animation functions like `chase()` and `rainbow()` that contained `delay()` calls, causing the entire program to pause during animations. This blocked:
- Sensor data reading
- Serial communication with the Lemon device
- Reed switch monitoring for power/reboot controls
- Energy usage calculations

## Solution Architecture

### Core Components

#### 1. Animation Event System
```cpp
enum AnimationType {
  ANIM_NONE = 0,
  ANIM_CHASE,
  ANIM_RAINBOW, 
  ANIM_THEATER_CHASE,
  ANIM_THEATER_CHASE_RAINBOW,
  ANIM_COLOR_WIPE,
  ANIM_FLASH_SINGLE,
  ANIM_ALL_OFF,
  ANIM_FILL_COLOR
};

struct AnimationEvent {
  AnimationType type;
  uint32_t color1, color2, color3, color4;
  int wait;
  int offset;
  int repeat_count;  // -1 for infinite, 0+ for specific count
  bool active;
};
```

#### 2. Animation State Management
```cpp
struct AnimationState {
  AnimationEvent current_event;
  uint32_t next_frame_time;
  int frame_counter;
  int step_counter;
  bool animation_active;
};
```

#### 3. Animation Queue System
```cpp
enum AnimationMode {
  ANIM_MODE_INTERRUPT = 0,  // New animation interrupts current one
  ANIM_MODE_QUEUE = 1       // New animation waits for current to finish
};

struct AnimationQueue {
  AnimationEvent events[MAX_ANIMATION_QUEUE];
  int head;
  int tail;
  int count;
  AnimationMode mode;
};
```

### Key Features

#### Queue Management with Two Modes
- **ANIM_MODE_INTERRUPT**: New animations immediately stop and replace current ones
- **ANIM_MODE_QUEUE**: New animations wait for the current one to finish

#### Non-Blocking Animation Functions
- **processChaseFrame()**: Async version of the original `chase()` function
- **processRainbowFrame()**: Async version of the original `rainbow()` function  
- **processFlashSingleFrame()**: Async version of `flashAndPauseNextPixel()`

#### Control Functions
- **startAnimation()**: Queue new animations with mode selection
- **stopCurrentAnimation()**: Immediately stop current animation
- **isAnimationActive()**: Check if animation is running
- **updateAnimations()**: Call this in your main loop

## Implementation Details

### Integration Points

#### setup() Function
```cpp
void setup() {
  // ... existing setup code ...
  
  // Initialize animation system
  initAnimationSystem();
}
```

#### loop() Function
```cpp
void loop() {
  accumulateEnergyUsage();
  sendSensorDataWhenReady();
  
  // Update animation system only if using async animations
  if (useAsyncAnimations) {
    updateAnimations();
  }
  
  // ... rest of loop logic ...
}
```

### Dual Mode Support

A global boolean flag allows switching between async and sync modes:

```cpp
bool useAsyncAnimations = true;  // Set to false to use old synchronous animations
```

Each animation case supports both modes:

```cpp
case LC_SEARCH_WIFI:
{
  if (useAsyncAnimations) {
    if (!isAnimationActive()) {
      startAnimation(ANIM_CHASE,
                    strip.Color(0,0,255),   // BLUE
                    strip.Color(0,0,100),
                    strip.Color(0,0,50),
                    strip.Color(0,0,0),
                    ledWait, globalOffset, -1);  // infinite repeat
    }
  } else {
    // Original synchronous code
    chase(globalOffset,
          strip.Color(0,0,255),   // BLUE
          strip.Color(0,0,100),
          strip.Color(0,0,50),
          strip.Color(0,0,0),
          ledWait);
  }
  break;
}
```

## Usage Examples

### Basic Animation Control

```cpp
// Start a chase animation that interrupts current animation
startAnimation(ANIM_CHASE, 
               strip.Color(255,0,0),     // color1 - bright red
               strip.Color(100,0,0),     // color2 - medium red  
               strip.Color(50,0,0),      // color3 - dim red
               strip.Color(0,0,0),       // color4 - off
               100,                      // wait time (ms)
               0,                        // offset
               -1,                       // repeat forever
               ANIM_MODE_INTERRUPT);

// Queue a rainbow that waits for current animation to finish
startAnimation(ANIM_RAINBOW, 0, 0, 0, 0, 50, 0, 2, ANIM_MODE_QUEUE);

// Stop current animation immediately
stopCurrentAnimation();

// Check if animation is running
if (isAnimationActive()) {
  // Animation is currently running
}
```

### Mode Switching

```cpp
// Use new asynchronous system (default)
useAsyncAnimations = true;

// Fall back to original synchronous system
useAsyncAnimations = false;
```

## Status Mapping

The system maps device status to specific animations:

| Status | Animation | Colors | Description |
|--------|-----------|--------|-------------|
| LC_STARTUP | ANIM_RAINBOW | Rainbow | Full rainbow cycle on startup |
| LC_SEARCH_WIFI | ANIM_CHASE | Blue gradient | Chase pattern while searching for WiFi |
| LC_FOUND_WIFI | ANIM_CHASE | Cyan gradient | Chase pattern when WiFi found |
| LC_NO_WIFI | ANIM_CHASE | Magenta gradient | Chase pattern when no WiFi |
| LC_NO_GPS | ANIM_CHASE | Red gradient | Chase pattern when no GPS |
| LC_NO_FIX | ANIM_CHASE | Yellow gradient | Chase pattern when no GPS fix |
| LC_GOOD_FIX | ANIM_CHASE | Green gradient | Chase pattern with good GPS fix |
| LC_NO_INTERNET | ANIM_CHASE | Blue/Magenta | Chase pattern when no internet |
| LC_ALL_OFF | ANIM_ALL_OFF | Off | Turn off all LEDs |
| LC_NONE | ANIM_FLASH_SINGLE | Red | Single LED flash (no communication) |

### Dive Mode Variants

When the `LC_DIVE_IN_PROGRESS` flag is set, animations switch to low-power single LED flash patterns to conserve battery.

## Benefits

✅ **Non-blocking**: Main loop continues running normally  
✅ **Flexible**: Choose interrupt vs queue behavior per animation  
✅ **State preservation**: Animation state persists between loop iterations  
✅ **Memory efficient**: Fixed-size queue, no dynamic allocation  
✅ **Easy to extend**: Add new animation types by implementing their frame functions  
✅ **Backward Compatible**: All original animation functions still work  
✅ **Easy Testing**: Switch modes with a single boolean flag  
✅ **Safe Fallback**: If async system has issues, instantly fall back to proven sync code

## Performance Impact

- **Memory Usage**: ~200 bytes for animation state and queue
- **CPU Overhead**: Minimal - only processes one frame per `updateAnimations()` call
- **Timing Accuracy**: Uses `millis()` for precise, non-blocking timing
- **Responsiveness**: Main loop can now run at full speed regardless of animation state

## Extending the System

### Adding New Animation Types

1. Add new enum value to `AnimationType`
2. Create a `processNewAnimationFrame()` function
3. Add case to `processAnimationFrame()` switch statement
4. Use `startAnimation()` with the new type

### Example New Animation

```cpp
// Add to AnimationType enum
ANIM_SPARKLE,

// Implement frame processing
void processSparkleFrame() {
  AnimationEvent* event = &anim_state.current_event;
  
  // Random sparkle logic here
  int randomPixel = random(strip.numPixels());
  strip.clear();
  strip.setPixelColor(randomPixel, event->color1);
  strip.show();
  
  anim_state.step_counter++;
  anim_state.next_frame_time = millis() + event->wait;
  
  // Continue until repeat count reached
  if (event->repeat_count >= 0 && anim_state.step_counter >= event->repeat_count) {
    anim_state.animation_active = false;
  }
}

// Add to processAnimationFrame()
case ANIM_SPARKLE:
  processSparkleFrame();
  break;
```

## Configuration

### Global Settings

```cpp
#define MAX_ANIMATION_QUEUE 5        // Maximum queued animations
bool useAsyncAnimations = true;      // Enable/disable async system
```

### Brightness Settings

```cpp
const int daytimeBrightness = 255;   // Full brightness for normal operation
const int nighttimeBrightness = 50;  // Reduced brightness for night
const int rainbowBrightness = 30;    // Low brightness for rainbow (power saving)
```

## Troubleshooting

### Common Issues

1. **Animations not starting**: Check `useAsyncAnimations` flag and ensure `updateAnimations()` is called in loop
2. **Jerky animations**: Ensure loop isn't blocked by other delay() calls
3. **Memory issues**: Reduce `MAX_ANIMATION_QUEUE` if needed
4. **Timing issues**: Check that `millis()` isn't overflowing (rare, happens every 49.7 days)

### Debug Functions

```cpp
// Check animation system status
Serial.print("Animation active: ");
Serial.println(isAnimationActive());
Serial.print("Queue count: ");
Serial.println(anim_queue.count);
Serial.print("Current type: ");
Serial.println(anim_state.current_event.type);
```

## Future Enhancements

- [ ] Add fade transitions between animations
- [ ] Implement animation priority levels
- [ ] Add animation callbacks for completion events
- [ ] Support for custom animation curves/easing
- [ ] Power usage optimization for battery operation
- [ ] Configuration via serial commands