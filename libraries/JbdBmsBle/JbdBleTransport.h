#pragma once
#include <queue>
#include <vector>

class JbdBleTransport {
public:
    void push(uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            buffer.push(data[i]);
        }
    }

    int available() {
        return buffer.size();
    }

    int read() {
        if (buffer.empty()) return -1;
        uint8_t v = buffer.front();
        buffer.pop();
        return v;
    }

    void clear() {
        while (!buffer.empty()) buffer.pop();
    }

    std::queue<uint8_t> buffer;
};