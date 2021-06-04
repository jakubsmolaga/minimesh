#ifndef BYTES_HPP
#define BYTES_HPP

struct ConstBytes
{
    const uint8_t *buf;
    const uint32_t len;
};

struct Bytes
{
    uint8_t *buf;
    uint32_t len;
    operator ConstBytes() const
    {
        return {buf, len};
    }
};

#endif
