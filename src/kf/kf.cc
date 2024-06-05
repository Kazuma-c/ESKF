#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "matplotlibcpp.h"

using namespace Eigen;
using namespace std;
namespace plt = matplotlibcpp;

int main() {
    VectorXd h(5);
    h << 4.9, 19.6, 44.1, 78.4, 122.5;//观测量
    MatrixXd X(3, 1);//状态变量，位置、速度、加速度
    X << 0, 0, 10;//初始状态变量的值
    MatrixXd Phi0(3, 3);//状态转移矩阵
    Phi0 << 1, 0.001, 0,
            0, 1, 0.001,
            0, 0, 1;//初始状态转移矩阵值    
    MatrixXd Phi = MatrixXd::Identity(3, 3);//单位阵    
    MatrixXd P = MatrixXd::Zero(3, 3);//状态变量的协方差
    P.diagonal() << 0, 0, 0.01;//主对角线的值
    MatrixXd Q = MatrixXd::Zero(3, 3);//状态变量噪声的协方差
    double R = 0.0001;//观测噪声的方差
    RowVectorXd H(3);//观测矩阵
    H << 1, 0, 0;//观测位置

    vector<MatrixXd> data(6, MatrixXd(3, 1));
    data[0] = X;

    for (int n = 1; n <= 5000; ++n) {
        Phi = Phi0 * Phi;//状态转移过程
        if (n % 1000 == 0) {
            MatrixXd Pkk = Phi * P * Phi.transpose() + Q;//预测状态变量的协方差
            double S = (H * Pkk * H.transpose())(0, 0) + R; // 计算S, 是1x1矩阵的第一个元素加R，观测的协方差
            MatrixXd K = Pkk * H.transpose() / S; // 标量除法，计算卡尔曼增益
            P = (MatrixXd::Identity(3, 3) - K * H) * Pkk*(MatrixXd::Identity(3, 3) - K * H) .transpose()+K*R*K.transpose();//更新状态变量的协方差
            MatrixXd Xkk = Phi * X;//状态变量预测值
            double z = h((n / 1000) - 1);//观测值
            X = Xkk + K * (z - (H * Xkk)(0, 0)); // 使用(0, 0)提取矩阵第一个元素，优化残差
            data[n / 1000] = X;
            Phi = MatrixXd::Identity(3, 3);//每次都要初始化是因为状态变量的值是在有观测值的时候更新的，在观测阶段保持的状态变量值在进行下一个状态的估计时，状态转移矩阵需要从零开始
        }
    }

    // Extract data for plotting
    vector<double> plot_data;
    for (const auto& d : data) {
        plot_data.push_back(d(2, 0));  // Third component
    }

    plt::plot(plot_data);
    plt::title("Kalman Filter Results");
    plt::show();

    return 0;
}
