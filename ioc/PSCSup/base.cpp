#include "base.hpp"

#include <core/assert.hpp>

#include <dbAccess.h>

ScanLockGuard::ScanLockGuard(dbCommon *db_common) : db_common_(db_common) {
    dbScanLock(db_common_);
}
ScanLockGuard::~ScanLockGuard() {
    unlock();
}

void ScanLockGuard::unlock() {
    if (db_common_ != nullptr) {
        dbScanUnlock(db_common_);
        db_common_ = nullptr;
    }
}

ScanLockGuard::ScanLockGuard(ScanLockGuard &&other) : db_common_(other.db_common_) {
    other.db_common_ = nullptr;
}
ScanLockGuard &ScanLockGuard::operator=(const ScanLockGuard &&other) {
    unlock();
    db_common_ = other.db_common_;
    return *this;
}


EpicsRecord::EpicsRecord(dbCommon *raw) :
    raw_(raw)
{
    init_async_process_callback(&async_callback_);
}

void EpicsRecord::set_private_data(dbCommon *raw, std::unique_ptr<EpicsRecord> &&record) {
    assert_true(raw->dpvt == nullptr);
    raw->dpvt = static_cast<void *>(record.release());
}
const EpicsRecord *EpicsRecord::get_private_data(const dbCommon *raw) {
    assert_true(raw->dpvt != nullptr);
    return (const EpicsRecord *)(raw->dpvt);
}
EpicsRecord *EpicsRecord::get_private_data(dbCommon *raw) {
    assert_true(raw->dpvt != nullptr);
    return (EpicsRecord *)(raw->dpvt);
}

bool EpicsRecord::is_process_active() const {
    return raw()->pact != FALSE;
}
void EpicsRecord::set_process_active(bool pact) {
    raw()->pact = pact ? TRUE : FALSE;
}

ScanLockGuard EpicsRecord::scan_lock() {
    return ScanLockGuard(raw());
}

void EpicsRecord::notify_async_process_complete() {
    auto rset = static_cast<struct typed_rset *>(raw()->rset);
    (*rset->process)(raw());
}

void EpicsRecord::schedule_async_process() {
    callbackRequest(&async_callback_);
}

void EpicsRecord::process_async() {
    const auto guard = scan_lock();
    process_sync();
    notify_async_process_complete();
}

void EpicsRecord::async_process_callback(epicsCallback *callback) {
    static_cast<EpicsRecord *>(callback->user)->process_async();
}

void EpicsRecord::init_async_process_callback(epicsCallback *callback) {
    callbackSetCallback(&EpicsRecord::async_process_callback, callback);
    callbackSetUser(static_cast<void *>(this), callback);
    callbackSetPriority(priorityLow, callback);
}

const dbCommon *EpicsRecord::raw() const {
    return raw_;
}
dbCommon *EpicsRecord::raw() {
    return raw_;
}

const Handler *EpicsRecord::handler() const {
    return handler_.get();
}
Handler *EpicsRecord::handler() {
    return handler_.get();
}

void EpicsRecord::process() {
    if (handler() == nullptr) {
        return;
    }
    if (handler()->is_async()) {
        if (!is_process_active()) {
            set_process_active(true);
            schedule_async_process();
        } else {
            set_process_active(false);
        }
    } else {
        process_sync();
    }
}

std::string_view EpicsRecord::name() const {
    return std::string_view(raw()->name);
}

void EpicsRecord::set_handler(std::unique_ptr<Handler> &&handler) {
    handler_ = std::move(handler);
}
