#include "EKF_Localization.h"

EKF_Localization::EKF_Localization() {
	// We approximate the process noise using a small constant
	this->setQ(0, 0, .0001); // Xr
	this->setQ(1, 1, .0001); // Yr
	this->setQ(2, 2, .0001); // Or

	// Same for measurement noise
	this->setR(0, 0, .0001);// Xodom
	this->setR(1, 1, .0001);// Yodom
	this->setR(2, 2, .0001);// Oodom
	this->setR(3, 3, .0001);// Ogyro
	this->setR(4, 4, .0001);// Omag
	this->setR(5, 5, .0001);// Xcam
	this->setR(6, 6, .0001);// Ycam
	this->setR(7, 7, .0001);// Ocam

}

void EKF_Localization::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
	// Process model is f(x) = x
	fx[0] = this->x[0];
	fx[1] = this->x[1];
	fx[2] = this->x[2];

	// So process model Jacobian is identity matrix
	F[0][0] = 1;
	F[1][1] = 1;
	F[2][2] = 1;

	// Measurement function
	hx[0] = this->x[0];
	hx[1] = this->x[1];
	hx[2] = this->x[2];
	hx[3] = this->x[2];
	hx[4] = this->x[2];
	hx[5] = this->x[0];
	hx[6] = this->x[1];
	hx[7] = this->x[2];


	// Jacobian of measurement function
	H[0][0] = 1;
	H[1][1] = 1;
	H[2][2] = 1;
	H[3][2] = 1;
	H[4][2] = 1;
	H[5][0] = 1;
	H[6][1] = 1;
	H[7][2] = 1;
}