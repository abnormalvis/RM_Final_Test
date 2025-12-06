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

        int fd = 0; // 读取终端设置
        if (tcgetattr(fd, &original_settings_) < 0)
        {
            throw std::runtime_error("Failed to get terminal attributes");
        }

        struct termios raw; // 获取到raw终端设置副本
        std::memcpy(&raw, &original_settings_, sizeof(struct termios));

        // 关闭规范模式和回显
        raw.c_lflag &= ~(ICANON | ECHO);    // 把相关位置为0
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;

        if (tcsetattr(fd, TCSANOW, &raw) < 0)
        {
            // 配置终端失败
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
            // 恢复终端设置
            tcsetattr(0, TCSANOW, &original_settings_);
            active_ = false;
        }
#endif
    }
    //  KeyboardInput 实现 
    KeyboardInput::KeyboardInput()
    {
    }
    KeyboardInput::~KeyboardInput()
    {
    }
    // 获取输入按键
    bool KeyboardInput::poll_key(char &key, int timeout_ms)
    {
#ifndef _WIN32
        /* Data structure describing a polling request.  */
        // struct pollfd
        // {
        //     int fd;			/* File descriptor to poll.  */
        //     short int events;		/* Types of events poller cares about.  */
        //     short int revents;		/* Types of events that actually occurred.  */
        // };
        struct pollfd ufd;
        ufd.fd = 0; // stdin
        ufd.events = POLLIN;
        // 通过 poll 非阻塞式检测是否有按键可读
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
