#pragma once

#include <cstdint>
#include <string>
#include <iostream>
#include <optional>

#include "result.hpp"
#include "format.hpp"

namespace io {

enum class ErrorKind {
    NotFound,
    InvalidData,
    UnexpectedEof,
    TimedOut,
    Other,
};

struct Error final {
    ErrorKind kind;
    std::string message;

    inline Error(ErrorKind kind) : kind(kind) {}
    inline Error(ErrorKind kind, std::string message) : kind(kind), message(message) {}
};

class StreamRead {
public:
    //! Try to read at most `len` bytes into `data` buffer.
    //! @return Number of bytes read or error.
    virtual Result<size_t, Error> stream_read(uint8_t *data, size_t len) = 0;
};

class StreamWrite {
public:
    //! Try to write at most `len` bytes from `data` buffer.
    //! @return Number of bytes written or error.
    virtual Result<size_t, Error> stream_write(const uint8_t *data, size_t len) = 0;
};

class StreamReadExact : public virtual StreamRead {
public:
    //! Try to read exactly `len` bytes into `data` buffer or return error.
    virtual Result<std::monostate, Error> stream_read_exact(uint8_t *data, size_t len) = 0;
};

class StreamWriteExact : public virtual StreamWrite {
public:
    //! Try to write exactly `len` bytes from `data` buffer or return error.
    virtual Result<std::monostate, Error> stream_write_exact(const uint8_t *data, size_t len) = 0;
};

class WriteFromStream {
public:
    //! Read bytes from given stream into `this`.
    //! @param len Number of bytes to read. If `nullopt` then read as much as possible.
    virtual Result<size_t, Error> write_from_stream(StreamRead &stream, std::optional<size_t> len = std::nullopt) = 0;
};

class ReadIntoStream {
public:
    //! Write bytes from `this` into given stream.
    //! @param len Number of bytes to write. If `nullopt` then write as much as possible.
    virtual Result<size_t, Error> read_into_stream(StreamWrite &stream, std::optional<size_t> len = std::nullopt) = 0;
};

} // namespace io

inline std::ostream &operator<<(std::ostream &o, const io::ErrorKind &ek) {
    switch (ek) {
    case io::ErrorKind::NotFound:
        o << "Not Found";
        break;
    case io::ErrorKind::UnexpectedEof:
        o << "Unexpected Eof";
        break;
    case io::ErrorKind::InvalidData:
        o << "Invalid Data";
        break;
    case io::ErrorKind::TimedOut:
        o << "Timed Out";
        break;
    case io::ErrorKind::Other:
        o << "Other";
        break;
    }
    return o;
}

inline std::ostream &operator<<(std::ostream &o, const io::Error &e) {
    return o << "io::Error(" << e.kind << ": " << e.message << ")";
}
