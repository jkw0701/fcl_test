#include <ros/ros.h>
#include <fcl/fcl.h>
#include "fcl_test/robot_model.h"

using namespace fcl;

robot::RobotModel * robot_;
VectorXd q(dof);
VectorXd q_lb(dof); // robot 7
VectorXd q_ub(dof); // robot 7


int main(int argc, char** argv) {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "fcl_test_node");
    ros::NodeHandle nh;

	robot_ = new robot::RobotModel(1); // 0: Manipulator, 1: Mobile Manipulaotr, 2: humanoid

    // 캡슐 두 개 생성: (반지름 0.5, 길이 2.0)
    Capsulef capsule1(0.5, .5);  // 첫 번째 캡슐
    Capsulef capsule2(0.5, .5);  // 두 번째 캡슐

    // 각 캡슐의 위치(변환 행렬) 설정
    Transform3f tf1 = Transform3f::Identity();
    Transform3f tf2 = Transform3f::Identity();
    tf1.translation() = Vector3f(0, 0, 0);  // 첫 번째 캡슐의 위치
    tf2.translation() = Vector3f(1.0, 1.0, 0);  // 두 번째 캡슐의 위치
    // Update end-effector transformation

	q_lb = -166.0 / 180.0 * M_PI * VectorXd(dof).setOnes();
	q_ub = -1.0 * q_lb;

	q_lb(1) = -101.0 / 180.0 * M_PI;
	q_ub(1) = 101.0 / 180.0 * M_PI;

	q_lb(3) = -176.0 / 180.0 * M_PI;
	q_ub(3) = -4.0 / 180.0 * M_PI;

	q_lb(5) = -1.0 / 180.0 * M_PI;
	q_ub(5) = 215 / 180.0 * M_PI;


	double jointrange;
	double r;
	VectorXd q_current(dof), qdot_current(dof);
	q_current.setZero();
    qdot_current.setZero();
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

    // CollisionObject로 캡슐 래핑
    std::shared_ptr<CollisionGeometryf> geom1 = std::make_shared<Capsulef>(capsule1);
    std::shared_ptr<CollisionGeometryf> geom2 = std::make_shared<Capsulef>(capsule2);

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
    if (dist >= 0) {
        std::cout << "Distance between capsules: " << dist << std::endl;

        // 가장 가까운 두 점 출력
        std::cout << "Nearest point on capsule 1: (" 
                  << result.nearest_points[0][0] << ", "
                  << result.nearest_points[0][1] << ", "
                  << result.nearest_points[0][2] << ")" << std::endl;

        std::cout << "Nearest point on capsule 2: (" 
                  << result.nearest_points[1][0] << ", "
                  << result.nearest_points[1][1] << ", "
                  << result.nearest_points[1][2] << ")" << std::endl;
    } else {
        std::cerr << "Negative distance calculation." << std::endl;
    }

    ros::spin();
    return 0;
}