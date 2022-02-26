#include "device.hpp"

#include <variant>
#include <cstring>

#include <core/assert.hpp>
#include <core/panic.hpp>
#include <core/convert.hpp>
#include <core/match.hpp>
#include <core/collections/vec.hpp>
#include <ipp.hpp>


void Device::recv_loop() {
    std::cout << "[app] Channel recv thread started" << std::endl;
    const auto timeout = std::chrono::milliseconds(100);

    channel.send(ipp::AppMsg{ipp::AppMsgConnect{}}, std::nullopt).unwrap(); // Wait forever
    std::cout << "[app] Connect signal sent" << std::endl;
    send_ready.notify_all();

    while (!this->done.load()) {
        auto result = channel.receive(timeout);
        if (result.is_err()) {
            auto err = result.unwrap_err();
            if (err.kind == io::ErrorKind::TimedOut) {
                continue;
            } else {
                // TODO: Use fmt
                std::stringstream text;
                text << err;
                panic("IO Error: " + text.str());
            }
        }
        auto incoming = result.unwrap();
        std::visit(
            overloaded{
                [&](ipp::McuMsgDinVal &&din_val) {
                    // std::cout << "Din updated: " << uint32_t(din_val.value) << std::endl;
                    din.value.store(din_val.value);
                    if (din.notify) {
                        din.notify();
                    }
                },
                [&](ipp::McuMsgAdcWf &&adc_wf_msg) {
                    auto &adc_wf = adc_wfs[adc_wf_msg.index];
                    if (adc_wf_msg.elements.size() > 0) {
                        adc_wf.last_value.store(adc_wf_msg.elements.back());
                    }
                    if (adc_wf.notify) {
                        std::lock_guard<std::mutex> lock(adc_wf.mutex);
                        adc_wf.wf_data.insert(adc_wf.wf_data.end(), adc_wf_msg.elements.begin(), adc_wf_msg.elements.end());

                        if (adc_wf.wf_data.size() >= adc_wf.wf_max_size) {
                            adc_wf.notify();
                        }
                    }
                },
                [&](ipp::McuMsgDacWfReq &&) {
                    has_dac_wf_req.store(true);
                    send_ready.notify_all();
                },
                [&](ipp::McuMsgDebug &&debug) { //
                    std::cout << "Device: " << debug.message << std::endl;
                },
                [&](ipp::McuMsgError &&error) {
                    std::cout << "Device Error (0x" << std::hex << int(error.code) << std::dec << "): " << error.message
                              << std::endl;
                },
                [&](auto &&) { unimplemented(); },
            },
            std::move(incoming.variant) //
        );
    }
    send_ready.notify_all();
}

void Device::send_loop() {
    std::cout << "[app] Channel send thread started" << std::endl;

    while (!this->done.load()) {
        std::unique_lock send_lock(send_mutex);
        auto status = send_ready.wait_for(send_lock, std::chrono::milliseconds(100));
        if (status == std::cv_status::timeout) {
            continue;
        }

        if (dout.update.exchange(false)) {
            uint8_t value = dout.value.load();
            std::cout << "[app] Send Dout value: " << uint32_t(value) << std::endl;
            channel.send(ipp::AppMsg{ipp::AppMsgDoutSet{uint8_t(value)}}, std::nullopt).unwrap();
        }
        if (has_dac_wf_req.load() == true) {
            auto tmp = dac_wf.tmp_buf;
            ipp::AppMsgDacWf dac_wf_msg;
            size_t max_count = (msg_max_len_ - dac_wf_msg.packed_size() - 1) / sizeof(int32_t);

            dac_wf.wf_data.read_array_into(tmp, max_count);

            if (!tmp.empty()) {
                has_dac_wf_req.store(false);
                dac_wf_msg.elements = std::move(tmp);

                assert_true(dac_wf_msg.packed_size() <= msg_max_len_ - 1);
                channel.send(ipp::AppMsg{std::move(dac_wf_msg)}, std::nullopt).unwrap();
                dac_wf.tmp_buf = std::move(tmp);
            }
        }
    }
}

Device::Device(std::unique_ptr<Channel> &&raw_channel, size_t msg_max_len) :
    msg_max_len_(msg_max_len),
    channel(std::move(raw_channel), msg_max_len) //
{
    done.store(true);
}
Device::~Device() {
    stop();
}

void Device::start() {
    done.store(false);
    send_worker = std::thread([this]() { this->send_loop(); });
    recv_worker = std::thread([this]() { this->recv_loop(); });
}

void Device::stop() {
    if (!done.load()) {
        done.store(true);
        send_worker.join();
        recv_worker.join();
    }
}

int32_t Device::read_adc(size_t index) {
    assert_true(index < ADC_COUNT);
    return adc_wfs[index].last_value.load();
}

void Device::write_dout(uint32_t value) {
    {
        constexpr uint32_t mask = 0xfu;
        std::lock_guard send_guard(send_mutex);
        if ((value & ~mask) != 0) {
            std::cout << "[app:warning] Ignoring extra bits in dout 4-bit mask: " << value << std::endl;
        }
        dout.value.store(uint8_t(value & mask));
        dout.update.store(true);
    }
    send_ready.notify_all();
}

uint32_t Device::read_din() {
    return din.value.load();
}
void Device::set_din_callback(std::function<void()> &&callback) {
    din.notify = std::move(callback);
}

void Device::write_dac_wf(const int32_t *wf_data, const size_t wf_len) {
    auto buf = dac_wf.wf_data.write_buffer();
    assert_true(buf->write_array_exact(wf_data, wf_len));
    send_ready.notify_all();
    if (dac_wf.request_next_wf) {
        dac_wf.request_next_wf();
    }
}

void Device::init_adc_wf(uint8_t index, size_t wf_max_size) {
    adc_wfs[index].wf_max_size = wf_max_size;
}

void Device::set_adc_wf_callback(size_t index, std::function<void()> &&callback) {
    assert_true(index < ADC_COUNT);
    adc_wfs[index].notify = std::move(callback);
}

const std::vector<int32_t> Device::read_adc_wf(size_t index) {
    auto &adc_wf = adc_wfs[index];

    std::lock_guard<std::mutex> lock(adc_wf.mutex);

    std::vector<int32_t> wf_data(adc_wf.wf_data.begin(), adc_wf.wf_data.begin() + adc_wf.wf_max_size);
    adc_wf.wf_data.erase(adc_wf.wf_data.begin(), adc_wf.wf_data.begin() + adc_wf.wf_max_size);

    return wf_data;
}

bool Device::dac_wf_req_flag() {
    return dac_wf.wf_data.write_buffer()->empty();
}

void Device::set_dac_wf_req_callback(std::function<void()> &&callback) {
    dac_wf.request_next_wf = std::move(callback);
}