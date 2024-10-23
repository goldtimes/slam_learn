#pragma once
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace timer {
class Timer {
   public:
    //定义一个结构体，保存函数运行的时间
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& func_name, double time_used) {
            func_name_ = func_name;
            time_usage_in_ms_.emplace_back(time_used);
        }
        std::string func_name_;
        std::vector<double> time_usage_in_ms_;
    };

    template <typename F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto start = std::chrono::steady_clock::now();
        //调用函数
        std::forward<F>(func)();
        auto end = std::chrono::steady_clock::now();
        // td::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() get time used s
        auto time_used_ms = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() * 1e3;
        if (records_.find(func_name) == records_.end()) {
            records_.insert({func_name, {func_name, time_used_ms}});
        } else {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used_ms);
        }
    }

    /// 打印记录的所有耗时
    static void PrintAll();

    /// 写入文件，方便作图分析
    static void DumpIntoFile(const std::string& file_name);

    /// 获取某个函数的平均执行时间
    static double GetMeanTime(const std::string& func_name);

    /// 清理记录
    static void Clear() {
        records_.clear();
    }

   private:
    static std::map<std::string, TimerRecord> records_;
};
}  // namespace timer