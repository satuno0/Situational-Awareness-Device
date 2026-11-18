#include <Arduino.h>
#include <Wire.h>
#include <esp32-hal-cpu.h>
#include <driver/gpio.h>
#include <esp_sleep.h>

#include <math.h>
#include <stdio.h>
#include <cstring>

#include "glove_pose_packet.h"

namespace {

constexpr uint8_t kSdaPin = 6;  // XIAO D4
constexpr uint8_t kSclPin = 7;  // XIAO D5
constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kDrvAddress = 0x5A;

constexpr uint8_t kWristTogglePin = 3;  // D1
constexpr uint8_t kButton1Pin = 4;      // D2
constexpr uint8_t kButton2Pin = 5;      // D3
constexpr uint8_t kButton3Pin = 21;     // D6

constexpr uint8_t kMpuPwrMgmt1 = 0x6B;
constexpr uint8_t kMpuAccelConfig = 0x1C;
constexpr uint8_t kMpuGyroConfig = 0x1B;
constexpr uint8_t kMpuAccelXoutH = 0x3B;

constexpr uint8_t kDrvMode = 0x01;
constexpr uint8_t kDrvWaveSeq1 = 0x04;
constexpr uint8_t kDrvWaveSeq2 = 0x05;
constexpr uint8_t kDrvGo = 0x0C;
constexpr uint8_t kDrvRatedVoltage = 0x16;
constexpr uint8_t kDrvOverdriveClamp = 0x17;
constexpr uint8_t kDrvFeedback = 0x1A;

constexpr uint32_t kDebounceMs = 25;
constexpr uint32_t kDoubleClickMs = 350;
constexpr uint32_t kMaxShortClickMs = 600;
constexpr uint32_t kPanicHoldMs = 3000;
constexpr uint32_t kImpactLockoutMs = 300;
constexpr uint32_t kPacketPeriodMs = 20;
constexpr uint32_t kLoopPeriodUs = 20000;
constexpr float kHighGThreshold = 2.5f;
constexpr float kMinHapticFrequencyHz = 1.0f;
constexpr float kMaxHapticFrequencyHz = 25.0f;

struct Button
{
    uint8_t pin;
    bool stable_high;
    bool last_raw_high;
    uint32_t changed_at_ms;
    uint32_t pressed_at_ms;
    bool pressed_event;
    bool released_event;
};

struct ClickGesture
{
    bool pending;
    uint32_t deadline_ms;
};

Button wrist_button{kWristTogglePin, true, true, 0, 0, false, false};
Button button1{kButton1Pin, true, true, 0, 0, false, false};
Button button2{kButton2Pin, true, true, 0, 0, false, false};
Button button3{kButton3Pin, true, true, 0, 0, false, false};

ClickGesture button1_click{};
ClickGesture button2_click{};

bool scan_armed = false;
bool panic_sent = false;
bool drv_calibrating = false;
bool mpu_ready = false;
uint8_t packet_sequence = 0;
uint8_t pending_flags = 0;
uint8_t pending_events = 0;
uint8_t pending_clicks = 0;
uint8_t pending_click_waveform = 1;
uint32_t next_click_ms = 0;
uint32_t geiger_interval_ms = 0;
uint32_t next_geiger_ms = 0;
uint8_t geiger_waveform = 1;
uint32_t last_packet_ms = 0;
uint32_t last_imu_ms = 0;
uint32_t impact_lockout_until_ms = 0;
float yaw_deg = 0.0f;
float pitch_deg = 0.0f;
char serial_line[80]{};
size_t serial_line_size = 0;

bool deadlineReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

bool inImpactLockout(uint32_t now_ms)
{
    return static_cast<int32_t>(now_ms - impact_lockout_until_ms) < 0;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* output, uint8_t count)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t received = static_cast<uint8_t>(Wire.requestFrom(address, count));
    if (received != count) return false;
    for (uint8_t i = 0; i < count; ++i)
        output[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

void startDrvCalibration()
{
    writeRegister(kDrvAddress, kDrvMode, 0x07);
    writeRegister(kDrvAddress, kDrvRatedVoltage, 0x50);
    writeRegister(kDrvAddress, kDrvOverdriveClamp, 0x89);
    writeRegister(kDrvAddress, kDrvFeedback, 0xB6);
    writeRegister(kDrvAddress, kDrvGo, 0x01);
    drv_calibrating = true;
}

void serviceDrvCalibration()
{
    if (!drv_calibrating) return;
    uint8_t go = 0;
    if (readRegisters(kDrvAddress, kDrvGo, &go, 1) && (go & 0x01) == 0)
    {
        writeRegister(kDrvAddress, kDrvMode, 0x00);
        drv_calibrating = false;
    }
}

void playWaveform(uint8_t waveform)
{
    if (drv_calibrating) return;
    writeRegister(kDrvAddress, kDrvMode, 0x00);
    writeRegister(kDrvAddress, kDrvWaveSeq1, waveform);
    writeRegister(kDrvAddress, kDrvWaveSeq2, 0);
    writeRegister(kDrvAddress, kDrvGo, 0x01);
}

void queueConfirmation(uint8_t clicks, uint8_t waveform, uint32_t now_ms)
{
    pending_clicks = clicks;
    pending_click_waveform = waveform;
    next_click_ms = now_ms;
}

void serviceConfirmations(uint32_t now_ms)
{
    if (drv_calibrating || pending_clicks == 0 ||
        !deadlineReached(now_ms, next_click_ms))
        return;
    playWaveform(pending_click_waveform);
    --pending_clicks;
    next_click_ms = now_ms + 100;
}

void initializeButton(Button& button, uint32_t now_ms)
{
    pinMode(button.pin, INPUT_PULLUP);
    button.stable_high = digitalRead(button.pin) != LOW;
    button.last_raw_high = button.stable_high;
    button.changed_at_ms = now_ms;
}

void sampleButton(Button& button, uint32_t now_ms)
{
    button.pressed_event = false;
    button.released_event = false;
    const bool raw_high = digitalRead(button.pin) != LOW;
    if (raw_high != button.last_raw_high)
    {
        button.last_raw_high = raw_high;
        button.changed_at_ms = now_ms;
    }
    if (raw_high == button.stable_high ||
        static_cast<uint32_t>(now_ms - button.changed_at_ms) < kDebounceMs)
        return;

    button.stable_high = raw_high;
    if (!raw_high)
    {
        button.pressed_at_ms = now_ms;
        button.pressed_event = true;
    }
    else
    {
        button.released_event = true;
    }
}

void resyncButtons(uint32_t now_ms)
{
    Button* const buttons[] = {&wrist_button, &button1, &button2, &button3};
    for (Button* button : buttons)
    {
        button->stable_high = digitalRead(button->pin) != LOW;
        button->last_raw_high = button->stable_high;
        button->changed_at_ms = now_ms;
        if (!button->stable_high) button->pressed_at_ms = now_ms;
        button->pressed_event = false;
        button->released_event = false;
    }
    if (!button3.stable_high) panic_sent = true;
    button1_click.pending = false;
    button2_click.pending = false;
}

void noteClick(ClickGesture& gesture, uint8_t single_event, uint32_t now_ms)
{
    if (gesture.pending && !deadlineReached(now_ms, gesture.deadline_ms))
    {
        gesture.pending = false;
        if (single_event == GLOVE_EVENT_RADAR_PING)
        {
            yaw_deg = 0.0f;
            pending_flags |= GLOVE_FLAG_ZERO_HEADING;
            queueConfirmation(2, 1, now_ms);
        }
        else
        {
            pending_events |= GLOVE_EVENT_TRAIL_TOGGLE;
            queueConfirmation(2, 2, now_ms);
        }
        return;
    }
    gesture.pending = true;
    gesture.deadline_ms = now_ms + kDoubleClickMs;
}

void serviceClickGestures(uint32_t now_ms)
{
    if (button1_click.pending && deadlineReached(now_ms, button1_click.deadline_ms))
    {
        button1_click.pending = false;
        pending_events |= GLOVE_EVENT_RADAR_PING;
        queueConfirmation(1, 1, now_ms);
    }
    if (button2_click.pending && deadlineReached(now_ms, button2_click.deadline_ms))
    {
        button2_click.pending = false;
        pending_events |= GLOVE_EVENT_QUICK_GLANCE;
        queueConfirmation(1, 2, now_ms);
    }
}

void handleButtons(uint32_t now_ms)
{
    if (inImpactLockout(now_ms))
    {
        resyncButtons(now_ms);
        return;
    }
    serviceClickGestures(now_ms);

    sampleButton(wrist_button, now_ms);
    sampleButton(button1, now_ms);
    sampleButton(button2, now_ms);
    sampleButton(button3, now_ms);

    if (wrist_button.pressed_event)
    {
        scan_armed = !scan_armed;
        queueConfirmation(scan_armed ? 2 : 1, scan_armed ? 1 : 2, now_ms);
    }

    if (button1.released_event &&
        static_cast<uint32_t>(now_ms - button1.pressed_at_ms) <= kMaxShortClickMs)
        noteClick(button1_click, GLOVE_EVENT_RADAR_PING, now_ms);
    if (button2.released_event &&
        static_cast<uint32_t>(now_ms - button2.pressed_at_ms) <= kMaxShortClickMs)
        noteClick(button2_click, GLOVE_EVENT_QUICK_GLANCE, now_ms);

    if (button3.pressed_event) panic_sent = false;
    if (!button3.stable_high && !panic_sent &&
        static_cast<uint32_t>(now_ms - button3.pressed_at_ms) >= kPanicHoldMs)
    {
        panic_sent = true;
        scan_armed = false;
        pending_events |= GLOVE_EVENT_PANIC_WIPE;
        queueConfirmation(3, 47, now_ms);
    }
    if (button3.released_event) panic_sent = false;

    serviceClickGestures(now_ms);
}

bool initializeMpu()
{
    if (!writeRegister(kMpuAddress, kMpuPwrMgmt1, 0x00)) return false;
    writeRegister(kMpuAddress, kMpuAccelConfig, 0x10);  // +/- 8 g
    writeRegister(kMpuAddress, kMpuGyroConfig, 0x00);   // +/- 250 deg/s
    return true;
}

bool sampleMpu(uint32_t now_ms)
{
    uint8_t data[14]{};
    if (!readRegisters(kMpuAddress, kMpuAccelXoutH, data, sizeof(data))) return false;
    const int16_t ax_raw = static_cast<int16_t>((data[0] << 8) | data[1]);
    const int16_t ay_raw = static_cast<int16_t>((data[2] << 8) | data[3]);
    const int16_t az_raw = static_cast<int16_t>((data[4] << 8) | data[5]);
    const int16_t gz_raw = static_cast<int16_t>((data[12] << 8) | data[13]);
    const float ax_g = static_cast<float>(ax_raw) / 4096.0f;
    const float ay_g = static_cast<float>(ay_raw) / 4096.0f;
    const float az_g = static_cast<float>(az_raw) / 4096.0f;
    const float magnitude_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (magnitude_g > kHighGThreshold)
        impact_lockout_until_ms = now_ms + kImpactLockoutMs;

    const float dt = last_imu_ms == 0 ? 0.02f :
                     static_cast<float>(now_ms - last_imu_ms) / 1000.0f;
    last_imu_ms = now_ms;
    yaw_deg += (static_cast<float>(gz_raw) / 131.0f) * dt;
    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;
    return true;
}

void emitPosePacket(uint32_t now_ms)
{
    const bool due = static_cast<uint32_t>(now_ms - last_packet_ms) >= kPacketPeriodMs;
    const bool button_event = wrist_button.pressed_event || wrist_button.released_event;
    if (!(scan_armed && due) && pending_flags == 0 && pending_events == 0 && !button_event)
        return;
    if (Serial.availableForWrite() < static_cast<int>(sizeof(GlovePosePacket))) return;

    GlovePosePacket packet{};
    packet.magic = GLOVE_PACKET_MAGIC;
    packet.seq = packet_sequence++;
    float wrapped_yaw = yaw_deg;
    while (wrapped_yaw > 180.0f) wrapped_yaw -= 360.0f;
    while (wrapped_yaw < -180.0f) wrapped_yaw += 360.0f;
    packet.yaw_cdeg = static_cast<int16_t>(lroundf(wrapped_yaw * 100.0f));
    packet.pitch_cdeg = static_cast<int16_t>(lroundf(pitch_deg * 100.0f));
    packet.btn_mask = wrist_button.stable_high ? 0 : 1;
    packet.flags = pending_flags;
    packet.reserved = pending_events;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 9; ++i) checksum ^= bytes[i];
    packet.checksum = checksum;
    Serial.write(bytes, sizeof(packet));
    pending_flags = 0;
    pending_events = 0;
    last_packet_ms = now_ms;
}

void stopDrv()
{
    writeRegister(kDrvAddress, kDrvWaveSeq1, 0);
    writeRegister(kDrvAddress, kDrvGo, 0);
}

void handleCoreHaptic(const char* line, uint32_t now_ms)
{
    float duty = 0.0f;
    float frequency = 0.0f;
    char pattern[20]{};
    if (sscanf(line, "HAPTIC,%f,%f,%19[^,\r\n]", &duty, &frequency, pattern) != 3)
        return;
    if (!isfinite(duty) || !isfinite(frequency) || frequency < 0.0f) return;
    if (duty <= 0.0f || std::strcmp(pattern, "SILENT") == 0)
    {
        geiger_interval_ms = 0;
        stopDrv();
        return;
    }

    geiger_waveform = 1;
    if (std::strcmp(pattern, "SAFE_TAGGED") == 0) geiger_waveform = 2;
    else if (std::strcmp(pattern, "PRESENCE_ONLY") == 0) geiger_waveform = 3;
    else if (std::strcmp(pattern, "COLLISION") == 0) geiger_waveform = 47;
    const float bounded_frequency = fminf(frequency, kMaxHapticFrequencyHz);
    if (bounded_frequency < kMinHapticFrequencyHz)
    {
        geiger_interval_ms = 0;
        stopDrv();
        return;
    }
    geiger_interval_ms = static_cast<uint32_t>(lroundf(1000.0f / bounded_frequency));
    next_geiger_ms = now_ms;
}

void pollCoreSerial(uint32_t now_ms)
{
    while (Serial.available() > 0)
    {
        const int value = Serial.read();
        if (value < 0) break;
        if (value == '\n')
        {
            serial_line[serial_line_size] = '\0';
            handleCoreHaptic(serial_line, now_ms);
            serial_line_size = 0;
        }
        else if (serial_line_size + 1 < sizeof(serial_line))
        {
            serial_line[serial_line_size++] = static_cast<char>(value);
        }
        else
        {
            serial_line_size = 0;
        }
    }
}

void serviceCoreHaptic(uint32_t now_ms)
{
    if (geiger_interval_ms == 0 || pending_clicks != 0 ||
        !deadlineReached(now_ms, next_geiger_ms))
        return;
    playWaveform(geiger_waveform);
    next_geiger_ms = now_ms + geiger_interval_ms;
}

void configureWakeSources()
{
    const uint8_t pins[] = {kWristTogglePin, kButton1Pin, kButton2Pin, kButton3Pin};
    esp_sleep_enable_gpio_wakeup();
    for (uint8_t pin : pins)
        gpio_wakeup_enable(static_cast<gpio_num_t>(pin), GPIO_INTR_LOW_LEVEL);
}

void enterLowPowerSleep()
{
    setCpuFrequencyMhz(10);
    esp_sleep_enable_timer_wakeup(kLoopPeriodUs);
    configureWakeSources();
    esp_light_sleep_start();
}

} // namespace

void setup()
{
    Serial.begin(115200);
    Wire.begin(kSdaPin, kSclPin);
    Wire.setClock(400000);

    const uint32_t now_ms = millis();
    initializeButton(wrist_button, now_ms);
    initializeButton(button1, now_ms);
    initializeButton(button2, now_ms);
    initializeButton(button3, now_ms);

    mpu_ready = initializeMpu();
    startDrvCalibration();
}

void loop()
{
    const uint32_t now_ms = millis();
    serviceDrvCalibration();
    pollCoreSerial(now_ms);
    if (static_cast<uint32_t>(now_ms - last_imu_ms) >= kPacketPeriodMs || last_imu_ms == 0)
    {
        if (mpu_ready) sampleMpu(now_ms);
        handleButtons(now_ms);
        emitPosePacket(now_ms);
    }
    serviceConfirmations(now_ms);
    serviceCoreHaptic(now_ms);

    if (!scan_armed && pending_clicks == 0 && geiger_interval_ms == 0 && !drv_calibrating)
        enterLowPowerSleep();
    else
        setCpuFrequencyMhz(80);
}
