#include <ros/ros.h>
#include <fcl/fcl.h>
#include "fcl_test/robot_model.h"
#include <vector>
#include <fstream>

using namespace fcl;

robot::RobotModel * robot_;
VectorXd q(dof);
VectorXd q_lb(dof); // robot 7
VectorXd q_ub(dof); // robot 7

float clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Reachability score를 RGB로 매핑
Eigen::Vector3i getColorFromScore(double score, double min_score, double max_score) {
    // Normalize 0 ~ 1
    float normalized_score = (score - min_score) / (max_score - min_score);
    normalized_score = clamp(normalized_score, 0.0f, 1.0f);

    int r = static_cast<int>(255 * normalized_score);
    int g = static_cast<int>(255 * normalized_score);
    int b = static_cast<int>(255 * (1.0f - normalized_score));

    return Eigen::Vector3i(r, g, b);
}

double sign(double x) {
    if(x >= 0.0) return 1.0;
    else return -1.0;
}

double computeSingularity(double x, double y, double phi, double r, double b) {
    double term = x * cos(phi) + y * sin(phi);
    double c = r / (2 * b);
    double singularity_value = 4 * b * b * pow(c, 4) * pow(term, 2);

    return sign(singularity_value) * singularity_value;
}

int main(int argc, char** argv) {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "fcl_test_node");
    ros::NodeHandle nh;

	robot_ = new robot::RobotModel(1); // 0: Manipulator, 1: Mobile Manipulaotr, 2: humanoid

    // 캡슐 두 개 생성: (반지름 0.5, 길이 2.0)
    Capsulef capsule1(0.3, 0.5552);  // 첫 번째 캡슐
    Capsulef capsule2(0.1, 0.107);  // 두 번째 캡슐

    // 각 캡슐의 위치(변환 행렬) 설정
    Transform3f tf1 = Transform3f::Identity();
    Transform3f tf2 = Transform3f::Identity();
    
    tf1.translation() = Vector3f(0, 0, 0.2405);  // 첫 번째 캡슐의 위치
    tf2.translation() = Vector3f(0.3656, 0.0, 1.596);  // 두 번째 캡슐의 위치

    // 첫 번째 캡슐을 y축으로 -90도 회전
    tf1.linear() = Eigen::Matrix3f::Identity();
    tf1.linear()(0, 0) = 0.0;
    tf1.linear()(0, 2) = 1.0;
    tf1.linear()(2, 0) = -1.0;
    tf1.linear()(2, 2) = 0.0;

    // Update end-effector transformation

    // CollisionObject로 캡슐 래핑
    std::shared_ptr<CollisionGeometryf> geom1 = std::make_shared<Capsulef>(capsule1);
    std::shared_ptr<CollisionGeometryf> geom2 = std::make_shared<Capsulef>(capsule2);

	q_lb = -166.0 / 180.0 * M_PI * VectorXd(dof).setOnes();
	q_ub = -1.0 * q_lb;

	// q_lb(1) = -101.0 / 180.0 * M_PI;
	// q_ub(1) = 101.0 / 180.0 * M_PI;

	// q_lb(3) = -176.0 / 180.0 * M_PI;
	// q_ub(3) = -4.0 / 180.0 * M_PI;

	// q_lb(5) = -1.0 / 180.0 * M_PI;
	// q_ub(5) = 215 / 180.0 * M_PI;
    
    q_lb(0) = -2.7437;
	q_ub(0) = 2.7437;

	q_lb(1) = -1.7837;
	q_ub(1) = 1.7837;

	q_lb(2) = -2.9007;
	q_ub(2) = 2.9007;

    q_lb(3) = -3.0421;
	q_ub(3) = -0.1518;

	q_lb(4) = -2.8065;
	q_ub(4) = 2.8065;

	q_lb(5) = -0.5445;
	q_ub(5) = 4.5169;

	q_lb(6) = -3.0159;
	q_ub(6) = 3.0159;


	double jointrange;
	double r;
	VectorXd q_current(dof), qdot_current(dof);
	q_current.setZero();
    qdot_current.setZero();

    // q_current(0) = 0;
    // q_current(1) = 1.57;
    // q_current(2) = 0;
    // q_current(3) = -1.57;
    // q_current(4) = 0;
    // q_current(5) = 1.57;
    // q_current(6) = 0;

    std::vector<Vector3f> nearest_points_capsule2;
    std::vector<Eigen::Vector3i> colors;
    std::vector<double> total_score;

    float epsilon = 0.0005;
    float bound = 0.15;

    // Reachability params
    double A = 525.9;
    double B = 0.575;
    double C = 100;

    double r_robot = 0.127;
    double b_robot = 0.635;

    Eigen::Vector3f manip_base_position(0.2776, 0.0, 1.003);

    double min_score = (-A * pow(0.4, 2) + C) * computeSingularity(0, b_robot, 0, r_robot, b_robot);
    double max_score = C * computeSingularity(b_robot, 0, 0, r_robot, b_robot);

    while (nearest_points_capsule2.size() < 10000)
    {
		    // random sampling of joint configuration
            for (int j = 0; j < dof; j++)
            {
                jointrange = q_ub(j) - q_lb(j);
                r = ((double)rand() / (double)RAND_MAX) * jointrange;
                q_current(j) = q_lb(j) + r;
            }
            robot_->getUpdateKinematics(q_current, qdot_current);
        
        fcl::Matrix3f Rot_ee;
        fcl::Vector3f Trs_ee;
        Vector3d Trsd_ee = robot_->getPosition(7);
        Matrix3d Rotd_ee = robot_->getOrientation(7);

        for (int i=0;i<3;i++)
            Trs_ee(i) = (float)Trsd_ee(i);

            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                {
                    Rot_ee(j, k) = (float)Rotd_ee(j, k);
                }

        tf2.linear() = Rot_ee;
        tf2.translation() = Trs_ee;
        Eigen::Vector3f ee_position = Trs_ee;

        CollisionObjectf obj1(geom1, tf1);
        CollisionObjectf obj2(geom2, tf2);

        // 거리 계산 요청 및 결과 객체 생성
        DistanceRequestf request;
        request.enable_nearest_points = true;  // 가장 가까운 두 점 활성화
        request.gjk_solver_type = GJKSolverType::GST_LIBCCD;  // 안정적인 GJK 솔버 사용

        DistanceResultf result;
        result.min_distance = std::numeric_limits<float>::infinity();  // 초기화

        // 거리 계산 수행
        float dist = distance(&obj1, &obj2, request, result);
        
        // 결과 출력
        if (abs(dist - bound) <= epsilon) {
            // Compute the reachability term
            double point_distance = (ee_position - manip_base_position).norm();
            double reachability_score = -A * pow((point_distance - B), 2) + C;

            // Compute the singularity term
            double singularity_score = computeSingularity(ee_position.x(), ee_position.y(), 0.0, r_robot, b_robot);

            // Eigen::Vector3i color = getColorFromScore(reachability_score * singularity_score, min_score, max_score);

            nearest_points_capsule2.push_back(result.nearest_points[1]);
            total_score.push_back(reachability_score * singularity_score * 10000);
            // colors.push_back(color);
            std::cerr << "Adding point :" << nearest_points_capsule2.size() << std::endl;
        }
    }
    ROS_INFO("Process finished.");

    // Save nearest points and RGB to a text file
    // std::ofstream outfile("DBB.txt");
    // for (size_t i = 0; i < nearest_points_capsule2.size(); ++i)
    // {
    //     const auto &point = nearest_points_capsule2[i];
    //     const auto &color = colors[i];
    //     outfile << point[0] << " " << point[1] << " " << point[2] << " "
    //             << color[0] << " " << color[1] << " " << color[2] << std::endl;
    // }
    // outfile.close();

    // Save nearest points and total scores to a text file
    std::ofstream outfile("DBB.txt");
    for (size_t i = 0; i < nearest_points_capsule2.size(); ++i) {
        const auto &point = nearest_points_capsule2[i];
        const auto &score = total_score[i];
        outfile << point[0] << " " << point[1] << " " << point[2] << " " << score << std::endl;
    }
    outfile.close();
    
    ROS_INFO("Saved %ld data points to DBB.txt", nearest_points_capsule2.size());

    ros::spin();
    return 0;
}