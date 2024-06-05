#include "eskf.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iomanip>
#include <fstream>

#include "io_utils.h"
#include "static_imu_init.h"
#include "utm_convert.h"
DEFINE_string(txt_path, "../data/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角(角度)");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, false, "是否加入轮速计信息");

    //存p，v，ba，bg数据
    void save_vec3(std::ofstream& fout, const Vec3d& v) {
    fout << v[0] << " " << v[1] << " " << v[2] << " ";
    };

    //存四元数
    void save_quat(std::ofstream& fout, const Quatd& q) {
    fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    void save_result(std::ofstream& fout, const sad::NavStated& save_state) {
    fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);//时间戳设置为18位有效数字，其他数据设置为9位有效数字
    save_vec3(fout, save_state.p_);
    save_quat(fout, save_state.R_.unit_quaternion());
    save_vec3(fout, save_state.v_);
    save_vec3(fout, save_state.bg_);
    save_vec3(fout, save_state.ba_);
    fout << std::endl;
    };
   
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc,&argv,true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    };

    //实例化一些需要用到的类，初始化imu，eskf，回调函数，RTK安装角
    sad::StaticIMUInit imu_init;  
    sad::ESKFD eskf;
    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x,FLAGS_antenna_pox_y);

    //导航结果保存路径
    std::ofstream fout("../data/gins.txt");

    bool imu_inited = false, gnss_inited = false;

    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    
    //匿名函数作为实参传入并进行预测过程，接着进行观测
    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
        
        if (!imu_init.InitSuccess()) {
            imu_init.AddIMU(imu);
            return;
        }

        if (!imu_inited) {
            // 读取初始零偏，设置ESKF
            sad::ESKFD::Options options;
            // 噪声由初始化器估计
            options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
            options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
            eskf.SetInitialConditions(options, imu_init.GetInitBg(),imu_init.GetInitBa(), imu_init.GetGravity());
            imu_inited = true;
            return;
        }

        if (!gnss_inited) {
            //等待有效的RTK数据
            return;
        }

        //接收到有效GNSS信号后开始预测
        eskf.Predict(imu);  

        //存入数据
        auto state = eskf.GetNominalState();   
 
        save_result(fout,state);

        usleep(1e3);
    })
    .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
        
        if (!imu_inited) {
                return;
            }
        sad::GNSS gnss_convert = gnss;
        if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos,FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
            return;
        }

        //去掉原点
        if (!first_gnss_set) {
            origin = gnss_convert.utm_pose_.translation();
            first_gnss_set = true;
        }
        gnss_convert.utm_pose_.translation() -= origin;
        eskf.ObserveGPS(gnss_convert);
        auto state = eskf.GetNominalState();
        save_result(fout,state);
        gnss_inited = true;
    })
    .SetOdomProcessFunc([&](const sad::Odom& odom) {
        //主要用于初始化
        imu_init.AddOdom(odom);
        if (FLAGS_with_odom && imu_inited && gnss_inited) {
            eskf.ObserveWheelSpeed(odom);
            //这两行也可以不用，低频里程计信号观测更新信息会融入到高频imu和GNSS观测更新信息中
            auto state = eskf.GetNominalState();
            save_result(fout,state);
        }
    })
    .Go();

    return 0;
}

