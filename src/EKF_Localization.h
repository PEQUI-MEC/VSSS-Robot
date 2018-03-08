// These must be defined before including TinyEKF.h
#define Nsta 3     // Three state values: Xr, Yr, Or;
#define Mobs 8     // Eight measurements: Xodom, Yodom, Oodom, Ogyro, Omag, Xcam, Ycam, Ocam;

#include <TinyEKF.h>

class EKF_Localization : public TinyEKF {

	public:
		EKF_Localization();

	protected:

		void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
};