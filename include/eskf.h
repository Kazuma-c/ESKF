#include "eigen_types.h"
#include "gnss.h"
#include "imu.h"
#include "math_utils.h"
#include "nav_state.h"
#include "odom.h"

#include <glog/logging.h>
#include <iomanip>

namespace sad {

template <typename S = double>
class ESKF {
    public:
    using SO3 = Sophus::SO3<S>;                         //定义旋转矩阵类型
    using Vec3T = Eigen::Matrix<S, 3, 1>;               //定义三维向量类型
    using Vec18T = Eigen::Matrix<S, 18, 1>;             //定义18维向量类型
    using Mat3T = Eigen::Matrix<S, 3, 3>;               //定义3x3矩阵类型
    using MotionNoiseT = Eigen::Matrix<S, 18 ,18>;      //定义18x18状态变量噪声协方差类型
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;          //定义3x3里程计噪声协方差类型
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;          //定义6x6GNSS噪声协方差类型
    using Mat18T = Eigen::Matrix<S, 18, 18>;            //定义18x18协方差类型
    using NavStateT = NavState<S>;                      //定义名义状态变量类型

    //包含各类噪声的结构体
    struct Options {
        Options() = default;                            //构造函数

        double imu_dt_ = 0.005;                         //IMU传输数据频率

        double gyro_var_ = 1e-5;                        //陀螺仪测量方差
        double acce_var_ = 1e-2;                        //加速度计测量方差
        double bias_gyro_var_ = 1e-6;                   //陀螺仪零偏方差
        double bias_acce_var_ = 1e-4;                   //加速度计零偏方差

        double odom_var_ = 0.5;                         //里程计测量标准差
        double odom_span_ = 0.1;                        //里程计测量间隔
        double wheel_radius_ = 0.155;                   //轮子半径
        double circle_pulse_ = 1024.0;                  //编码器每圈脉冲数

        double gnss_pos_noise_ = 0.1;                   //GNSS位置噪声标准差
        double gnss_height_noise_ = 0.1;                //GNSS高度噪声标准差
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  //GNSS旋转噪声标准差

        bool update_bias_gyro_ = true;                  //是否更新陀螺零偏
        bool update_bias_acce_ = true;                  //是否更新加速度计零偏
    };

    ESKF(Options option = Options()) : options_(option) {
        BuildNoise(option);
    }

    //设置初始条件，噪声协方差项配置、初始零偏、重力、初始误差状态变量协方差
    void SetInitialConditions(Options options, const Vec3T& init_bg, const Vec3T& init_ba, const Vec3T& gravity = Vec3T(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_P = Mat18T::Identity() * 1e-4; 
    }

    //预测过程函数的声明
    bool Predict(const IMU& imu);

    //里程计观测和更新过程函数的声明
    bool ObserveWheelSpeed(const Odom& odom);

    //GPS观测和更新过程函数的声明
    bool ObserveGPS(const GNSS& gnss);
    bool ObserveSE3(const SE3& pose);

    //获取名义状态变量（上一时刻名义状态变量和上一时刻的误差状态变量之和）
    NavStateT GetNominalState() const {
        return NavState(current_time_, R_, p_, v_, bg_, ba_);
    }

    //获取变换矩阵
    SE3 GetNominalSE3() const {
        return SE3(R_,p_);
    }

    //设置名义状态变量状态
    void SetX(const NavStated& x, const Vec3d& grav){
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }

    //设置误差状态变量协方差矩阵
    void SetCov(const Mat18T& cov) {
        cov_P = cov;
    }

    //获取重力
    Vec3d GetGravity() const {
        return g_;
    }

    private:
    //创建初始状态变量协方差矩阵P、里程计噪声协方差矩阵、GNSS噪声协方差矩阵
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        Q_.diagonal() << 0, 0, 0, ev, ev, ev, et, et, et, eg, eg, eg, ea, ea, ea, 0, 0, 0;

        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_cov.diagonal() << o2, o2, o2;

        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_cov.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;    
    }

    //更新名义状态变量，重置误差状态变量为0
    void UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));
        bg_ += dx_.template block<3, 1>(9, 0);
        ba_ += dx_.template block<3, 1>(12, 0);
        g_ += dx_.template block<3, 1>(15, 0);

        dx_.setZero();
    }

    //一些成员变量

    ////名义状态变量
    double current_time_ = 0.0;
    Vec3T p_ = Vec3T::Zero();
    Vec3T v_ = Vec3T::Zero();
    SO3 R_;
    Vec3T bg_ = Vec3T::Zero();
    Vec3T ba_ = Vec3T::Zero();
    Vec3T g_{0, 0, -9.8};

    Vec18T dx_ = Vec18T::Zero();                    //误差状态
    Mat18T cov_P = Mat18T::Identity();              //误差状态协方差矩阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();         //误差状态噪声协方差矩阵
    OdomNoiseT odom_noise_cov = OdomNoiseT::Zero(); //里程计噪声协方差矩阵
    GnssNoiseT gnss_noise_cov = GnssNoiseT::Zero(); //GNSS噪声协方差矩阵

    bool first_gnss_ = true;

    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

//名义状态和误差状态协方差矩阵的预测
template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);

    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }

    //名义状态变量的预测过程
    Vec3T new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    Vec3T new_v = v_ +R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;

    //误差状态雅可比矩阵F
    Mat18T F = Mat18T::Identity();                                                 // 主对角线
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // p 对 v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v 对 ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v 对 g
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta 对 bg

    //误差状态变量协方差矩阵的预测过程
    cov_P = F * cov_P.eval() * F.transpose() + Q_;

    //更新时间
    current_time_ = imu.timestamp_;
    return true;
}

    //里程计观测更新过程
template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);

    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Mat3T::Identity();

    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K = cov_P * H.transpose() * (H * cov_P * H.transpose() + odom_noise_cov).inverse();

    // velocity obs
    //计算左轮速度
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    //计算右轮速度
    double velo_r =
        options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    //车辆平均速度
    double average_vel = 0.5 * (velo_l + velo_r);
    //载体坐标系下的速度，只有车辆前进方向
    Vec3T vel_odom(average_vel, 0.0, 0.0);
    //转换到导航坐标系下
    Vec3T vel_world = R_ * vel_odom;

    //更新误差状态变量
    dx_ = K * (vel_world - v_);

    //更新误差状态变量协方差矩阵
    cov_P = (Mat18T::Identity() - K * H) * cov_P;

    //将误差状态变量并入名义状态变量，并将误差状态变量置零
    UpdateAndReset();
    return true;
}

template <typename S>
bool ESKF<S>::ObserveGPS(const GNSS& gnss) {
    assert(gnss.unix_time_ >= current_time_);

    //用第一个GNSS数据初始化旋转矩阵和位置
    if (first_gnss_) {
        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }

    assert(gnss.heading_valid_);
    ObserveSE3(gnss.utm_pose_);
    current_time_ = gnss.unix_time_;

    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose){
    //观测噪声协方差矩阵
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();
    H.template block<3, 3>(3, 6) = Mat3T::Identity();

    //GNSS观测卡尔曼增益
    Eigen::Matrix<S, 18, 6> K = cov_P * H.transpose() * (H * cov_P * H.transpose() + gnss_noise_cov).inverse();

    //残差
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();

    //更新误差状态变量
    dx_ = K * innov;

    //更新误差状态变量协方差矩阵
    cov_P = (Mat18T::Identity() - K * H) * cov_P * (Mat18T::Identity() - K * H).transpose() + K * gnss_noise_cov * K.transpose();

    //将误差状态变量并入名义状态变量，并将误差状态变量置零
    UpdateAndReset();
    return true;
}
}