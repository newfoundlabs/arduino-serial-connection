#pragma once

#include <termios.h>

#include <filesystem>

class Serial {

public:
    Serial(const std::filesystem::path& path, const int baud = B9600) noexcept : m_path{path}, m_baud{baud} {}

    void openPortAndAttemptLock();

    std::string readData();

    void closePort();

private:
    const std::filesystem::path m_path;
    const int m_baud;
    
    int m_serialPort{ -1 };
};
