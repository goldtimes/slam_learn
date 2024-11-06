#pragma once

#include <map>
#include <memory>
#include <string>
#include "common/eigen_types.hh"
#include "common/nav_state.hh"
#include "common/sensors/gnss.hh"
#include "lidar_slam/ch8/lio-iekf/lio_iekf.hh"
#include "lidar_slam/ch9/keyframe.hh"

namespace slam_learn::ieskf_lio {
class LioIEKF;
}  // namespace slam_learn::ieskf_lio

namespace slam_learn::mapping {
using namespace ieskf_lio;
class Frontend {
   public:
    struct Options {};

    // 从yaml文件中读取数据路径
    explicit Frontend(const std::string& config_yaml);

    // 初始化，创建LIO对象，检查数据存在性
    bool Init();

    /// 运行前端
    void Run();

   private:
    /// 看某个state是否能提取到RTK pose
    void ExtractKeyFrame(const NavStated& state);

    /// 确定某个关键帧的GPS pose
    void FindGPSPose(std::shared_ptr<Keyframe> kf);

    /// 保存各个关键帧位姿，点云在生成时已经保存
    void SaveKeyframes();

    /// 提取RTK地图原点并移除此原点
    void RemoveMapOrigin();

   private:
    std::shared_ptr<Keyframe> last_kf_ = nullptr;
    // 抽取的所有关键帧
    std::map<unsigned long, std::shared_ptr<Keyframe>> keyframes_;
    // lio
    std::shared_ptr<LioIEKF> lio_ = nullptr;
    //  配置文件
    std::string config_yaml_;
    std::string bag_path_;
    std::string lio_yaml_;
    // gnss数据
    std::map<double, GNSSPtr> gnss_datas_;
    unsigned long kf_id_ = 0;

    Vec3d map_origin_ = Vec3d::Zero();

    // 关键帧的阈值
    double kf_distance_th_ = 1.0;
    double kf_angular_th_ = 10.0;
};
}  // namespace slam_learn::mapping