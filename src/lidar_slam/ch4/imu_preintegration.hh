#include "common/eigen_types.hh"
#include "common/nav_state.hh"
#include "common/sensors/imu.hh"

namespace slam_learn {
class IMUPreintegration {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    struct Options {
        Options() {
        }
        Vec3d init_bg_ = Vec3d::Zero();  // 初始零偏
        Vec3d init_ba_ = Vec3d::Zero();  // 初始零偏
        double noise_gyro_ = 1e-2;       // 陀螺噪声，标准差
        double noise_acce_ = 1e-1;       // 加计噪声，标准差
    };

    IMUPreintegration(Options options = Options());

    /**
     * 插入新的IMU数据
     * @param imu   imu 读数
     * @param dt    时间差
     */
    void Integrate(const IMU &imu, double dt);

    /**
     * 从某个起始点开始预测积分之后的状态
     * @param start 起始时时刻状态
     * @return  预测的状态
     */
    NavStated Predict(const NavStated &start, const Vec3d &grav = Vec3d(0, 0, -9.81)) const;

    /// 获取修正之后的观测量，bias可以与预积分时期的不同，会有一阶修正
    SO3 GetDeltaRotation(const Vec3d &bg);
    Vec3d GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba);
    Vec3d GetDeltaPosition(const Vec3d &bg, const Vec3d &ba);

   public:
    // 预积分的间隔时间
    double dt_;

    // 预积分量
    Vec3d dp_ = Vec3d::Zero();
    Vec3d dv_ = Vec3d::Zero();
    SO3 dR_;

    // 零偏
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    // 协方差矩阵
    Mat9d cov_ = Mat9d::Identity();
    // 噪声矩阵
    Mat6d noise_gyro_acc_ = Mat6d::Zero();
    // A,B矩阵

    // 对零偏的jacobian
    Mat3d dR_dbg_ = Mat3d::Zero();
    Mat3d dP_dbg_ = Mat3d::Zero();
    Mat3d dP_dba_ = Mat3d::Zero();
    Mat3d dV_dbg_ = Mat3d::Zero();
    Mat3d dV_dba_ = Mat3d::Zero();
};
}  // namespace slam_learn