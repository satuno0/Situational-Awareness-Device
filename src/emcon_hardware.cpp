#include "hal_pinout.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>

#if defined(__linux__)
#include <cerrno>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
#include <gpiod.h>
#endif

namespace sa {
namespace {

constexpr std::size_t kMaxI2cMessageSize = 4096;

uint64_t saturatingAdd(uint64_t value, uint64_t increment)
{
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    return value > maximum - increment ? maximum : value + increment;
}

struct DebouncedInput
{
    bool candidate = false;
    bool stable = false;
    uint64_t candidate_since_ms = 0;

    void reset(bool value, uint64_t timestamp_ms)
    {
        candidate = value;
        stable = value;
        candidate_since_ms = timestamp_ms;
    }

    bool update(bool value, uint64_t timestamp_ms, uint32_t debounce_ms)
    {
        if (value != candidate)
        {
            candidate = value;
            candidate_since_ms = timestamp_ms;
        }
        if (candidate != stable && timestamp_ms >= candidate_since_ms &&
            timestamp_ms - candidate_since_ms >= debounce_ms)
        {
            stable = candidate;
        }
        return stable;
    }
};

#if defined(__linux__)
bool configureI2cFd(int fd, uint32_t timeout_ms, std::string& error)
{
    const unsigned long timeout_units = std::max(1u, (timeout_ms + 9u) / 10u);
    if (ioctl(fd, I2C_TIMEOUT, timeout_units) < 0)
    {
        error = "cannot set I2C transaction timeout: " + std::string(std::strerror(errno));
        return false;
    }
    if (ioctl(fd, I2C_RETRIES, 0) < 0)
    {
        error = "cannot disable I2C adapter retries: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
}
#endif

} // namespace

struct EmconHardware::Impl
{
    EmconHardwareConfig config;
    LidInterlockCallbacks lid_callbacks;
    EmconHardwareStatus status;
    uint64_t last_timestamp_ms = 0;
    uint64_t pulse_deadline_ms = 0;
    uint64_t pending_click_deadline_ms = 0;
    uint64_t snooze_deadline_ms = 0;
    bool have_timestamp = false;
    bool pulse_active = false;
    bool pending_click = false;
    bool lid_action_fired = false;
    bool button_was_pressed = false;
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    gpiod_chip* chip = nullptr;
    gpiod_line* sw1 = nullptr;
    gpiod_line* q1 = nullptr;
    gpiod_line* button1 = nullptr;
    gpiod_line* lid = nullptr;
#endif
    DebouncedInput auto_input;
    DebouncedInput button_input;
};

EmconHardware::EmconHardware() : impl_(std::make_unique<Impl>()) {}
EmconHardware::~EmconHardware() { stop(); }

bool EmconHardware::start(const EmconHardwareConfig& config,
                          const LidInterlockCallbacks& lid_callbacks,
                          uint64_t timestamp_ms, std::string& error)
{
    stop();
    if (config.gpio_chip.empty() || config.input_debounce_ms == 0 ||
        config.double_click_ms == 0 || config.manual_sweep_ms == 0 ||
        config.advisory_snooze_ms == 0 ||
        lid_callbacks.zeroize_ram_buffers == nullptr || lid_callbacks.drop_luks_keys == nullptr)
    {
        error = "invalid EMCON configuration or missing lid-interlock callbacks";
        return false;
    }
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    impl_->config = config;
    impl_->lid_callbacks = lid_callbacks;
    impl_->chip = gpiod_chip_open(config.gpio_chip.c_str());
    if (impl_->chip == nullptr)
    {
        error = "cannot open GPIO chip " + config.gpio_chip + ": " + std::strerror(errno);
        return false;
    }

    impl_->sw1 = gpiod_chip_get_line(impl_->chip, hal_pinout::gpio_sw1_auto_manual);
    impl_->q1 = gpiod_chip_get_line(impl_->chip, hal_pinout::gpio_radar_power_gate);
    impl_->button1 = gpiod_chip_get_line(impl_->chip, hal_pinout::gpio_button1);
    impl_->lid = gpiod_chip_get_line(impl_->chip, hal_pinout::gpio_lid_reed);
    if (impl_->sw1 == nullptr || impl_->q1 == nullptr || impl_->button1 == nullptr ||
        impl_->lid == nullptr)
    {
        error = "cannot access one or more EMCON GPIO lines";
        stop();
        return false;
    }

    const unsigned int pull_up = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    if (gpiod_line_request_input_flags(impl_->sw1, "sa-emcon-sw1", pull_up) < 0 ||
        gpiod_line_request_input_flags(impl_->button1, "sa-emcon-button1", pull_up) < 0 ||
        gpiod_line_request_input_flags(impl_->lid, "sa-emcon-lid", pull_up) < 0 ||
        gpiod_line_request_output(impl_->q1, "sa-radar-power", 0) < 0)
    {
        error = "cannot configure EMCON GPIO lines with pull-ups and radar gate low: " +
                std::string(std::strerror(errno));
        stop();
        return false;
    }

    const int auto_value = gpiod_line_get_value(impl_->sw1);
    const int button_value = gpiod_line_get_value(impl_->button1);
    const int lid_value = gpiod_line_get_value(impl_->lid);
    if (auto_value < 0 || button_value < 0 || lid_value < 0)
    {
        error = "cannot read initial EMCON GPIO state: " + std::string(std::strerror(errno));
        stop();
        return false;
    }
    impl_->auto_input.reset(auto_value != 0, timestamp_ms);
    impl_->button_input.reset(button_value == 0, timestamp_ms);
    impl_->status = {};
    impl_->status.auto_mode = impl_->auto_input.stable;
    impl_->status.button_pressed = impl_->button_input.stable;
    impl_->button_was_pressed = impl_->status.button_pressed;
    impl_->status.lid_open = lid_value != 0;
    impl_->last_timestamp_ms = timestamp_ms;
    impl_->have_timestamp = true;
    impl_->lid_action_fired = false;
    if (impl_->status.lid_open)
    {
        impl_->lid_callbacks.zeroize_ram_buffers(impl_->lid_callbacks.context);
        impl_->lid_callbacks.drop_luks_keys(impl_->lid_callbacks.context);
        impl_->lid_action_fired = true;
    }
    impl_->status.radar_powered = impl_->status.auto_mode;
    if (gpiod_line_set_value(impl_->q1, impl_->status.radar_powered ? 1 : 0) < 0)
    {
        error = "cannot set initial radar power gate: " + std::string(std::strerror(errno));
        stop();
        return false;
    }
    error.clear();
    return true;
#else
    (void)timestamp_ms;
    (void)lid_callbacks;
    error = "EMCON GPIO hardware requires Linux and libgpiod 1.x";
    return false;
#endif
}

bool EmconHardware::poll(uint64_t timestamp_ms, EmconHardwareStatus& status, std::string& error)
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    if (impl_->chip == nullptr)
    {
        error = "EMCON hardware is not running";
        return false;
    }
    if (impl_->have_timestamp && timestamp_ms < impl_->last_timestamp_ms)
    {
        error = "EMCON timestamps must be nondecreasing";
        return false;
    }

    const int auto_value = gpiod_line_get_value(impl_->sw1);
    const int button_value = gpiod_line_get_value(impl_->button1);
    const int lid_value = gpiod_line_get_value(impl_->lid);
    if (auto_value < 0 || button_value < 0 || lid_value < 0)
    {
        error = "cannot read EMCON GPIO state: " + std::string(std::strerror(errno));
        return false;
    }

    const bool lid_open = lid_value != 0;
    if (lid_open && !impl_->lid_action_fired)
    {
        impl_->lid_callbacks.zeroize_ram_buffers(impl_->lid_callbacks.context);
        impl_->lid_callbacks.drop_luks_keys(impl_->lid_callbacks.context);
        impl_->lid_action_fired = true;
    }
    else if (!lid_open)
    {
        impl_->lid_action_fired = false;
    }
    impl_->status.lid_open = lid_open;

    const bool previous_auto = impl_->auto_input.stable;
    impl_->status.auto_mode = impl_->auto_input.update(auto_value != 0, timestamp_ms,
                                                       impl_->config.input_debounce_ms);
    impl_->status.button_pressed = impl_->button_input.update(button_value == 0, timestamp_ms,
                                                              impl_->config.input_debounce_ms);
    if (previous_auto != impl_->status.auto_mode)
    {
        impl_->pending_click = false;
        impl_->pulse_active = false;
        impl_->button_was_pressed = impl_->status.button_pressed;
    }

    if (impl_->status.button_pressed && !impl_->button_was_pressed)
    {
        if (impl_->status.auto_mode)
        {
            if (impl_->pending_click && timestamp_ms <= impl_->pending_click_deadline_ms)
            {
                impl_->snooze_deadline_ms =
                    saturatingAdd(timestamp_ms, impl_->config.advisory_snooze_ms);
                impl_->pending_click = false;
            }
            else
            {
                impl_->pending_click = true;
                impl_->pending_click_deadline_ms =
                    saturatingAdd(timestamp_ms, impl_->config.double_click_ms);
            }
        }
        else
        {
            impl_->pulse_active = true;
            impl_->pulse_deadline_ms = saturatingAdd(timestamp_ms, impl_->config.manual_sweep_ms);
        }
    }
    impl_->button_was_pressed = impl_->status.button_pressed;

    if (impl_->pending_click && timestamp_ms > impl_->pending_click_deadline_ms)
        impl_->pending_click = false;
    if (impl_->pulse_active && timestamp_ms >= impl_->pulse_deadline_ms)
        impl_->pulse_active = false;

    const bool radar_powered = impl_->status.auto_mode || impl_->pulse_active;
    if (radar_powered != impl_->status.radar_powered &&
        gpiod_line_set_value(impl_->q1, radar_powered ? 1 : 0) < 0)
    {
        error = "cannot update radar power gate: " + std::string(std::strerror(errno));
        return false;
    }
    impl_->status.radar_powered = radar_powered;
    impl_->status.advisory_snoozed = timestamp_ms < impl_->snooze_deadline_ms;
    impl_->last_timestamp_ms = timestamp_ms;
    status = impl_->status;
    error.clear();
    return true;
#else
    (void)timestamp_ms;
    (void)status;
    error = "EMCON GPIO hardware requires Linux and libgpiod 1.x";
    return false;
#endif
}

void EmconHardware::stop()
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    if (impl_->q1 != nullptr) (void)gpiod_line_set_value(impl_->q1, 0);
    if (impl_->sw1 != nullptr) gpiod_line_release(impl_->sw1);
    if (impl_->q1 != nullptr) gpiod_line_release(impl_->q1);
    if (impl_->button1 != nullptr) gpiod_line_release(impl_->button1);
    if (impl_->lid != nullptr) gpiod_line_release(impl_->lid);
    if (impl_->chip != nullptr) gpiod_chip_close(impl_->chip);
    impl_->sw1 = nullptr;
    impl_->q1 = nullptr;
    impl_->button1 = nullptr;
    impl_->lid = nullptr;
    impl_->chip = nullptr;
#endif
    impl_->status = {};
    impl_->have_timestamp = false;
    impl_->pulse_active = false;
    impl_->pending_click = false;
    impl_->lid_action_fired = false;
    impl_->button_was_pressed = false;
}

bool EmconHardware::running() const
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    return impl_->chip != nullptr;
#else
    return false;
#endif
}

struct FrontTofPodControl::Impl
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    gpiod_chip* chip = nullptr;
    gpiod_line* s4_xshut = nullptr;
#endif
};

FrontTofPodControl::FrontTofPodControl() : impl_(std::make_unique<Impl>()) {}
FrontTofPodControl::~FrontTofPodControl() { stop(); }

bool FrontTofPodControl::start(std::string_view gpio_chip, TofAddressSetter set_s3_address,
                               void* context, std::string& error)
{
    stop();
    if (gpio_chip.empty() || set_s3_address == nullptr)
    {
        error = "front ToF pod requires a GPIO chip and S3 address setter";
        return false;
    }
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    const std::string chip_path(gpio_chip);
    impl_->chip = gpiod_chip_open(chip_path.c_str());
    if (impl_->chip == nullptr)
    {
        error = "cannot open GPIO chip " + chip_path + ": " + std::strerror(errno);
        return false;
    }
    impl_->s4_xshut = gpiod_chip_get_line(impl_->chip, hal_pinout::gpio_tof_s4_xshut);
    if (impl_->s4_xshut == nullptr)
    {
        error = "cannot access S4 XSHUT GPIO line";
        stop();
        return false;
    }
    if (gpiod_line_request_output(impl_->s4_xshut, "sa-tof-s4-xshut", 0) < 0)
    {
        error = "cannot hold S4 XSHUT low: " + std::string(std::strerror(errno));
        stop();
        return false;
    }
    if (!set_s3_address(context, hal_pinout::tof_s3_default_address,
                        hal_pinout::tof_s3_address, error))
    {
        if (error.empty()) error = "S3 I2C address assignment failed";
        return false;
    }
    if (gpiod_line_set_value(impl_->s4_xshut, 1) < 0)
    {
        error = "cannot release S4 XSHUT: " + std::string(std::strerror(errno));
        return false;
    }
    error.clear();
    return true;
#else
    (void)context;
    error = "front ToF pod GPIO requires Linux and libgpiod 1.x";
    return false;
#endif
}

void FrontTofPodControl::stop()
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    if (impl_->s4_xshut != nullptr)
    {
        (void)gpiod_line_set_value(impl_->s4_xshut, 0);
        gpiod_line_release(impl_->s4_xshut);
    }
    if (impl_->chip != nullptr) gpiod_chip_close(impl_->chip);
    impl_->s4_xshut = nullptr;
    impl_->chip = nullptr;
#endif
}

bool FrontTofPodControl::running() const
{
#if defined(__linux__) && defined(SA_HAS_LIBGPIOD)
    return impl_->chip != nullptr && impl_->s4_xshut != nullptr;
#else
    return false;
#endif
}

struct AsyncI2cBus::Impl
{
    struct Request
    {
        uint64_t id = 0;
        uint8_t address = 0;
        std::vector<uint8_t> write_data;
        std::size_t read_size = 0;
    };

    I2cBusConfig config;
    std::atomic<bool> active{false};
    std::thread worker;
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Request> requests;
    std::unordered_map<uint64_t, I2cResult> results;
    uint64_t next_id = 1;
};

AsyncI2cBus::AsyncI2cBus() : impl_(std::make_unique<Impl>()) {}
AsyncI2cBus::~AsyncI2cBus() { stop(); }

bool AsyncI2cBus::start(const I2cBusConfig& config, std::string& error)
{
    stop();
    if (config.device.empty() || config.timeout_ms == 0 || config.reconnect_delay_ms == 0 ||
        config.max_queued_transactions == 0)
    {
        error = "invalid asynchronous I2C configuration";
        return false;
    }
#if defined(__linux__)
    impl_->config = config;
    impl_->next_id = 1;
    impl_->active = true;
    try
    {
        impl_->worker = std::thread([state = impl_.get()] {
        int fd = -1;
        std::string open_error;
        auto closeBus = [&] {
            if (fd >= 0) ::close(fd);
            fd = -1;
        };
        auto openBus = [&]() -> bool {
            fd = ::open(state->config.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
            {
                open_error = std::strerror(errno);
                return false;
            }
            if (!configureI2cFd(fd, state->config.timeout_ms, open_error))
            {
                closeBus();
                return false;
            }
            return true;
        };

        while (state->active.load())
        {
            Impl::Request request;
            {
                std::unique_lock<std::mutex> lock(state->mutex);
                state->ready.wait(lock, [state] {
                    return !state->active.load() || !state->requests.empty();
                });
                if (!state->active.load()) break;
                request = std::move(state->requests.front());
                state->requests.pop_front();
            }

            I2cResult result{};
            result.transaction_id = request.id;
            if (fd < 0 && !openBus())
            {
                result.error = "cannot open or configure I2C bus " + state->config.device +
                               ": " + open_error;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->results.emplace(result.transaction_id, std::move(result));
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(state->config.reconnect_delay_ms));
                continue;
            }

            std::vector<uint8_t> read_data(request.read_size);
            i2c_msg messages[2]{};
            uint32_t message_count = 0;
            if (!request.write_data.empty())
            {
                messages[message_count].addr = request.address;
                messages[message_count].flags = 0;
                messages[message_count].len =
                    static_cast<uint16_t>(request.write_data.size());
                messages[message_count].buf = request.write_data.data();
                ++message_count;
            }
            if (request.read_size != 0)
            {
                messages[message_count].addr = request.address;
                messages[message_count].flags = I2C_M_RD;
                messages[message_count].len = static_cast<uint16_t>(request.read_size);
                messages[message_count].buf = read_data.data();
                ++message_count;
            }
            i2c_rdwr_ioctl_data transaction{};
            transaction.msgs = messages;
            transaction.nmsgs = static_cast<uint32_t>(message_count);
            const int transferred = ioctl(fd, I2C_RDWR, &transaction);
            if (transferred < 0)
            {
                result.error = "I2C transaction failed: " + std::string(std::strerror(errno));
                closeBus();
            }
            else if (transferred != static_cast<int>(message_count))
            {
                result.error = "I2C transaction transferred " + std::to_string(transferred) +
                               " of " + std::to_string(message_count) + " messages";
                closeBus();
            }
            else
            {
                result.success = true;
                result.read_data = std::move(read_data);
            }
            std::lock_guard<std::mutex> lock(state->mutex);
            state->results.emplace(result.transaction_id, std::move(result));
        }
        closeBus();
        });
    }
    catch (const std::system_error& exception)
    {
        impl_->active = false;
        error = "cannot start asynchronous I2C worker: " + std::string(exception.what());
        return false;
    }
    error.clear();
    return true;
#else
    (void)config;
    error = "I2C hardware is supported only on Linux";
    return false;
#endif
}

void AsyncI2cBus::stop()
{
    if (!impl_) return;
    impl_->active = false;
    impl_->ready.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->requests.clear();
    impl_->results.clear();
}

bool AsyncI2cBus::submit(uint8_t address, std::span<const uint8_t> write_data,
                         std::size_t read_size, uint64_t& transaction_id, std::string& error)
{
    if (!impl_->active.load())
    {
        error = "asynchronous I2C bus is not running";
        return false;
    }
    if (address > 0x7F || (write_data.empty() && read_size == 0) ||
        write_data.size() > kMaxI2cMessageSize || read_size > kMaxI2cMessageSize)
    {
        error = "invalid I2C address or transaction size";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->requests.size() + impl_->results.size() >=
        impl_->config.max_queued_transactions)
    {
        error = "asynchronous I2C queue is full";
        return false;
    }
    if (impl_->next_id == 0)
    {
        error = "asynchronous I2C transaction identifier exhausted";
        return false;
    }
    Impl::Request request{};
    request.id = impl_->next_id++;
    request.address = address;
    request.write_data.assign(write_data.begin(), write_data.end());
    request.read_size = read_size;
    transaction_id = request.id;
    impl_->requests.push_back(std::move(request));
    impl_->ready.notify_one();
    error.clear();
    return true;
}

bool AsyncI2cBus::tryTakeResult(uint64_t transaction_id, I2cResult& result)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto found = impl_->results.find(transaction_id);
    if (found == impl_->results.end()) return false;
    result = std::move(found->second);
    impl_->results.erase(found);
    return true;
}

bool AsyncI2cBus::running() const { return impl_->active.load(); }

} // namespace sa
