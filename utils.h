#pragma once

#include <iostream>
#include <regex>
#include <string>

#include "log_exceptions.h"

enum class Device { AUTO = -1, CPU = 0, CUDA = 1 };

bool IsGPU(Device device) {
    if (device == Device::AUTO) {
#ifdef CUDA_ENABLED
        return true;
#else
        return false;
#endif
    } else {
        return static_cast<bool>(device);
    }
}

void VerifySiftGPUParams(const bool use_gpu) {
#ifndef CUDA_ENABLED
    if (use_gpu) {
        THROW_EXCEPTION(std::invalid_argument,
                        "Cannot use Sift GPU without CUDA support; "
                        "set device='auto' or device='cpu'.")
    }
#endif
}