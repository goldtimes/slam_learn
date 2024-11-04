#include "timer.hh"

namespace slam_learn::timer {
//
std::map<std::string, Timer::TimerRecord> Timer::records_;
/// 打印记录的所有耗时
void Timer::PrintAll() {
}

/// 写入文件，方便作图分析
void Timer::DumpIntoFile(const std::string& file_name) {
}

/// 获取某个函数的平均执行时间
double Timer::GetMeanTime(const std::string& func_name) {
    return 0.0;
}

}  // namespace slam_learn::timer