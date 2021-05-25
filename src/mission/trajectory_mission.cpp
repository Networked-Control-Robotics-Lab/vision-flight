#include <stdio.h>
#include "trajectory_mission.hpp"

void TrajectoryManager::add(trajectory_t& x, trajectory_t& y, trajectory_t& z)
{
	trajectory3d_t traj3d;
	traj3d.x = x;
	traj3d.y = y;
	traj3d.z = z;
	this->trajs.push_back(traj3d); 
}

void TrajectoryManager::clear()
{
	this->trajs.clear();
}

void TrajectoryManager::get_trajectory(int index, trajectory_t& x, trajectory_t& y, trajectory_t& z)
{
	x = this->trajs.at(index).x;
	y = this->trajs.at(index).y;
	z = this->trajs.at(index).z;
}

void TrajectoryManager::print_trajectory(float *traj_coeff, int coeff_size)
{
	for(int i = 0; i < coeff_size; i++) {
		if(i == (coeff_size - 1)) {
			printf("%f\n\r", traj_coeff[i]);
		} else {
			printf("%f, ", traj_coeff[i]);
		}
	}
}

void TrajectoryManager::print_list()
{
	for(int i = 0; i < this->trajs.size(); i++) {
		/* position */
		printf("trajectory #%d:\n\r"
                       "position coefficients:\n\r"
		       "x: ", i);
		print_trajectory(this->trajs.at(i).x.pos_coeff, 8);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.pos_coeff, 8);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.pos_coeff, 8);

		/* velocity */
		printf("velocity coefficients:\n\r"
                       "x: ");
		print_trajectory(this->trajs.at(i).x.vel_coeff, 7);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.vel_coeff, 7);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.vel_coeff, 7);

		/* acceleration */
		printf("acceleration coefficients:\n\r"
                       "x: ");
		print_trajectory(this->trajs.at(i).x.accel_coeff, 6);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.accel_coeff, 6);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.accel_coeff, 6);

		printf("---\n\r");
	}
}

bool TrajectoryManager::send()
{
}
