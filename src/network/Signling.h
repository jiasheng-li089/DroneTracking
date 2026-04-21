#pragma once

#include <string>

class Signaling {
public:
    virtual ~Signaling() = default;
    virtual std::string exchange_offer(const std::string& offer) = 0;
    virtual void end() = 0;
};