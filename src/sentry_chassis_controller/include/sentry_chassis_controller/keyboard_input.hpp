/*
 * KeyboardInput - 键盘输入抽象层
 * 功能：封装终端设置、非阻塞按键读取、跨平台支持
 * 用途：将底层终端操作与上层业务逻辑解耦
 */

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
        struct termios original_settings_;
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

        /*
         * 轮询键盘输入
         * @param key 输出参数：读取到的按键字符
         * @param timeout_ms 超时时间（毫秒），-1表示永久等待
         * @return true 有按键输入，false 超时或错误
         */
        bool poll_key(char &key, int timeout_ms);

        /*
         * 检查是否有按键可读（不消耗按键）
         * @param timeout_ms 超时时间（毫秒）
         * @return true 有按键可读
         */
        bool has_key(int timeout_ms);

        /*
         * 将字符转换为小写
         */
        static char to_lower(char c);

    private:
        TerminalGuard terminal_guard_;
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_KEYBOARD_INPUT_HPP
