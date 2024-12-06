#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_config.h>
#include "fcl_test/fwd.h"

#include <iostream>
#include <memory>
#include <fstream>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;

namespace robot {

		 enum class Type : unsigned int {
		 	Manipulator = 0,
		 	MobileManipulator = 1,
		 	Humanoid = 2
		 };

		class RobotModel
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				RobotModel(int robottype);
			~RobotModel();

			void getUpdateKinematics(const VectorXd & q, const VectorXd & qdot);
			Vector3d & getPosition(const int & frame_id) {
				Position(frame_id);
				return m_pos_;
			}
			Matrix3d & getOrientation(const int & frame_id) {
				Orientation(frame_id);
				return m_Ori_;
			}
			Transform3d & getTransformation(const int & frame_id) {
				Transformation(frame_id);
				return m_Trans_;
			}

		private:
			void setRobot();
			void Position(const int & frame_id);
			void Orientation(const int & frame_id);
			void Transformation(const int & frame_id);			
            /////////////////////////////////////////////////////////////////
			unsigned int body_id_sub[dof];
			unsigned int body_id_sim[dof];
			Math::Vector3d com_pos_sub[dof];
			Math::Vector3d com_pos_dis[6];
			Body body_sub[dof];
			Body body_sim[dof];

			shared_ptr<Model> model_;
			Body body_[dof];
			Body virtual_body_[6];
			Body base_;

			Joint joint_[dof];
			Joint virtual_joint_[6];

			VectorXd q_;
			VectorXd qdot_;

			VectorXd q_real_;
			VectorXd qdot_real_;

			double mass_[dof];
			Math::Vector3d axis_[dof];
			Math::Vector3d inertia_[dof];
			Math::Vector3d joint_position_global_[dof];
			Math::Vector3d joint_position_local_[dof];
			Math::Vector3d com_position_[dof];
			Math::SpatialVector p_dot_;

			// Obstacle avoidance
			Math::Vector3d joint_pos[dof + 1];
			Math::Vector3d joint_pos_diff[dof];
			Math::Vector3d joint_to_obs[dof];
			double dot_product[dof];
			double dot_product_joint[dof];
			Math::Vector3d closest_on_link[dof];
			VectorXd distance_vector;
			MatrixXd Closest_Jaco;
			MatrixXd Jaco_pos;
			MatrixXd Jaco_temp;
			Vector3d Obs_avoid;
			double shortest_dis;
			Vector3d Point_vel;
			Vector3d Closest_point;
			Vector3d Unit_Task;

			Math::VectorNd q_rbdl_;
			Math::VectorNd qdot_rbdl_;
			Math::VectorNd qddot_rbdl_;
			Math::VectorNd tau_;

			unsigned int body_id_[dof];
			unsigned int base_id_; // virtual joint
			unsigned int virtual_body_id_[6];

			MatrixXd m_Mass_mat_;
			VectorXd m_NLE_;
			Vector3d m_pos_;
			Matrix3d m_Ori_;
			MatrixXd m_J_;
			MatrixXd m_selection_;
			MatrixXd m_selection_dot_, m_Mass_virtual_mat_;

			Transform3d m_Trans_;
			Transform3d m_base_;

			Vector3d m_mob_pos;
			int m_robot_type_;

			double m_manipulability_[2];
			VectorXd m_J_manipulability_[2];
			MatrixXd m_J_EE[2];

		protected:
			unsigned int m_nv_;
			unsigned int m_na_;
			unsigned int m_nq_;

		};
}

#endif