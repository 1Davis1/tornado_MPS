#pragma once

#include <functional>
#include <string_view>
#include <memory>

// Abstract record interface.
class Record {
public:
    virtual std::string_view name() const = 0;
};

// Abstract record handler.
class Handler {
private:
    const bool async_;

public:
    Handler(bool async) : async_(async) {}
    virtual ~Handler() = default;

    inline bool is_async() const {
        return async_;
    }
};
