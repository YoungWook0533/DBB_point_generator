#include "fcl_test/robot_model.h"
#include <vector>


using namespace std;

robot::RobotModel::RobotModel(int robottype) {
	// model_ = new Model();
	// model_->gravity = Eigen::Vector3d(0., 0., -9.81);


	q_rbdl_.resize(dof);
	qdot_rbdl_.resize(dof);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();

	qddot_rbdl_.resize(dof);
	qddot_rbdl_.setZero();

	m_Ori_.resize(3, 3);
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();

	tau_.resize(m_na_ + 5);
	tau_.setZero();

	q_real_.resize(m_nv_);
	q_real_.setZero();
	qdot_real_.resize(m_nv_);
	qdot_real_.setZero();

	distance_vector.resize(dof);
	Closest_Jaco.resize(6, dof);
	Jaco_temp.resize(6, dof);
	Jaco_pos.resize(3, dof);

	setRobot();
}
robot::RobotModel::~RobotModel() {

}

void robot::RobotModel::setRobot() {
	model_ = make_shared<Model>();
	model_->gravity = Vector3d(0., 0, -9.81);

	for (int i = 0; i < dof; i++) {
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(1.0, 1.0, 1.0);
	}

	// Front 
	axis_[0] = Eigen::Vector3d::UnitZ();
	axis_[1] = Eigen::Vector3d::UnitY();
	axis_[2] = Eigen::Vector3d::UnitZ();
	axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = Eigen::Vector3d::UnitZ();
	axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = -1.0*Eigen::Vector3d::UnitZ();
	
	// Front Panda_husky
	// joint_position_global_[0] = Eigen::Vector3d(0.3502, 0.0, 0.7081) ;
	// joint_position_global_[1] = Eigen::Vector3d(0.3502, 0.0, 0.7080) ;
	// joint_position_global_[2] = Eigen::Vector3d(0.3503, 0.0, 1.0240) ;
	// joint_position_global_[3] = Eigen::Vector3d(0.4328, 0.0, 1.0240) ;
	// joint_position_global_[4] = Eigen::Vector3d(0.3504, 0.0, 1.4080) ;
	// joint_position_global_[5] = Eigen::Vector3d(0.3504, 0.0, 1.4080) ;
	// joint_position_global_[6] = Eigen::Vector3d(0.4384, 0.0, 1.4080) ;

	// Front fr3_summit
	joint_position_global_[0] = Eigen::Vector3d(0.2776, 0.0, 1.003) ;
	joint_position_global_[1] = Eigen::Vector3d(0.2776, 0.0, 1.003) ;
	joint_position_global_[2] = Eigen::Vector3d(0.2776, 0.0, 1.319) ;
	joint_position_global_[3] = Eigen::Vector3d(0.3601, 0.0, 1.319) ;
	joint_position_global_[4] = Eigen::Vector3d(0.2776, 0.0, 1.703) ;
	joint_position_global_[5] = Eigen::Vector3d(0.2776, 0.0, 1.703) ;
	joint_position_global_[6] = Eigen::Vector3d(0.3656, 0.0, 1.703) ;

	joint_position_local_[0] = joint_position_global_[0];

	for (int i = 1; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i - 1];

	// Front
	com_position_[0] = Vector3d(0.3502, -0.0345, 0.6325) ;
	com_position_[1] = Vector3d(0.3504, 0.0345, 0.7844) ;
	com_position_[2] = Vector3d(0.3837, 0.0267, 0.9826) ;
	com_position_[3] = Vector3d(0.3834, -0.0265, 1.0665) ;
	com_position_[4] = Vector3d(0.3517, 0.0424, 1.2993) ;
	com_position_[5] = Vector3d(0.3925, -0.0102, 1.4232) ;
	// Set as center of the capsule
	com_position_[6] = Vector3d(0.3656, 0.0, 1.596) ;

	for (int i = 0; i < dof; i++)
		com_position_[i] -= joint_position_global_[i];

	for (int i = 0; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}

	////////////////////////////////////////////////////////////////

}

void robot::RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) { // for mobile
	q_rbdl_ = q;
	qdot_rbdl_ = qdot;
	qddot_rbdl_.setZero();
	UpdateKinematics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_);
}
void robot::RobotModel::Position(const int & frame_id) { // for mobile
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
}
void robot::RobotModel::Orientation(const int & frame_id) { // for mobile
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();

}
void robot::RobotModel::Transformation(const int & frame_id) { // for mobile
	Position(frame_id);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}