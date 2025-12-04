/*
 * KeyboardInput - 键盘输入抽象层实现
 */

#include "sentry_chassis_controller/keyboard_input.hpp"

#ifndef _WIN32
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <stdexcept>
#endif

namespace sentry_chassis_controller
{

    //  TerminalGuard 实现 
    TerminalGuard::TerminalGuard() : active_(false)
    {
        setup();
    }
    TerminalGuard::~TerminalGuard()
    {
        restore();
    }
    void TerminalGuard::setup()
    {
#ifndef _WIN32
        if (active_)
            return;

        int fd = 0; // stdin
        if (tcgetattr(fd, &original_settings_) < 0)
        {
            throw std::runtime_error("Failed to get terminal attributes");
        }

        struct termios raw;
        std::memcpy(&raw, &original_settings_, sizeof(struct termios));

        // 关闭规范模式和回显
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;

        if (tcsetattr(fd, TCSANOW, &raw) < 0)
        {
            throw std::runtime_error("Failed to set terminal attributes");
        }

        active_ = true;
#endif
    }

    void TerminalGuard::restore()
    {
#ifndef _WIN32
        if (active_)
        {
            tcsetattr(0, TCSANOW, &original_settings_);
            active_ = false;
        }
#endif
    }
    //  KeyboardInput 实现 
    KeyboardInput::KeyboardInput()
    {
        // 构造时 TerminalGuard 已自动 setup
    }
    KeyboardInput::~KeyboardInput()
    {
        // 析构时 TerminalGuard 会自动 restore
    }
    bool KeyboardInput::poll_key(char &key, int timeout_ms)
    {
#ifndef _WIN32
        struct pollfd ufd;
        ufd.fd = 0; // stdin
        ufd.events = POLLIN;

        int result = poll(&ufd, 1, timeout_ms);

        if (result < 0)
        {
            // poll 错误
            return false;
        }

        if (result == 0)
        {
            // 超时
            return false;
        }

        // 有数据可读
        if (read(0, &key, 1) < 0)
        {
            return false;
        }

        return true;
#else
        // Windows 平台暂不支持
        (void)key;
        (void)timeout_ms;
        return false;
#endif
    }

    bool KeyboardInput::has_key(int timeout_ms)
    {
#ifndef _WIN32
        struct pollfd ufd;
        ufd.fd = 0;
        ufd.events = POLLIN;

        int result = poll(&ufd, 1, timeout_ms);
        return (result > 0);
#else
        (void)timeout_ms;
        return false;
#endif
    }

    char KeyboardInput::to_lower(char c)
    {
        if (c >= 'A' && c <= 'Z')
            return c - 'A' + 'a';
        return c;
    }

} // namespace sentry_chassis_controller
