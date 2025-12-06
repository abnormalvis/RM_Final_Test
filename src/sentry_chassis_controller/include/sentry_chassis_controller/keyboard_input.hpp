#ifndef SENTRY_CHASSIS_CONTROLLER_KEYBOARD_INPUT_HPP
#define SENTRY_CHASSIS_CONTROLLER_KEYBOARD_INPUT_HPP

#ifndef _WIN32
#include <termios.h>
#endif

namespace sentry_chassis_controller
{

    /*
     * 终端设置管理器
     * 职责：在构造时设置终端为原始模式，在析构时恢复
     */
    class TerminalGuard
    {
    public:
        TerminalGuard();
        ~TerminalGuard();

        // 禁止拷贝和赋值
        TerminalGuard(const TerminalGuard &) = delete;
        TerminalGuard &operator=(const TerminalGuard &) = delete;

        // 手动设置和恢复（用于特殊场景）
        void setup();
        void restore();
        bool is_active() const { return active_; }

    private:
#ifndef _WIN32
        struct termios original_settings_;  // 原始终端设置
#endif
        bool active_;
    };

    /*
     * 键盘输入读取器
     * 职责：提供非阻塞的按键读取接口
     */
    class KeyboardInput
    {
    public:
        KeyboardInput();
        ~KeyboardInput();

        bool poll_key(char &key, int timeout_ms);
        bool has_key(int timeout_ms);
        static char to_lower(char c);

    private:
        TerminalGuard terminal_guard_;
    };

} // namespace sentry_chassis_controller

#endif
