/*
 * ik.h
 *
 *  Created on: Apr 12, 2022
 *      Author: yuxuan
 */

#ifndef INC_IK_H_
#define INC_IK_H_

# include <math.h>
# define BIGARM_LEN 180
# define FOREARM_LEN 180
# define REDUCTION_RATIO 12.5
# define BIG_0 0.2774533464102067
# define FORE_0 1.6255593064102065
# define BIG_1 2.8641393071795864
# define FORE_1 1.5160333471795866



struct Arm
{
public:
	enum Dir{Left, Right} type;
	double base_x_;
	double base_y_;
	double base_theta_;


private:
	double x_, y_, dis_, theta_total_, theta_big_, theta_fore_;
public:
//	Arm(Arm::Dir dir, double ref_fore, double ref_big, double cen_fore, double cen_big): type(dir) {
//		base_x = cos(cen_big) * BIGARM_LEN + cos(cen_big + cen_fore) * FOREARM_LEN;
//		base_y = sin(cen_big) * BIGARM_LEN + sin(cen_big + cen_fore) * FOREARM_LEN;
//		base_theta = atan2(
//				sin(ref_big) * BIGARM_LEN + sin(ref_big + ref_fore) * FOREARM_LEN - base_y,
//				cos(ref_big) * BIGARM_LEN + cos(ref_big + ref_fore) * FOREARM_LEN - base_x);
//	}
//	Arm() : type(Left), base_x(52.8), base_y(165.5), base_theta(0){}
	Arm(Arm::Dir dir, double x, double y, double theta):
		type(dir), base_x_(x), base_y_(y), base_theta_(theta){}
	void GetArmAngle(double& bigarm_angle, double& forearm_angle, double pos_x, double pos_y) {
		x_ = base_x_ + pos_x * cos(base_theta_) - pos_y * sin(base_theta_);
		y_ = base_y_ + pos_y * cos(base_theta_) + pos_x * sin(base_theta_);
		dis_ = sqrt(x_ * x_ + y_ * y_);
		theta_total_ = (M_PI - atan2(y_, x_));
		theta_big_ = acos((dis_ * dis_ + BIGARM_LEN * BIGARM_LEN - FOREARM_LEN * FOREARM_LEN) / (2 * BIGARM_LEN * dis_));
		theta_fore_ = acos((dis_ * dis_ - BIGARM_LEN * BIGARM_LEN + FOREARM_LEN * FOREARM_LEN) / (2 * FOREARM_LEN * dis_));
		if (this->type == Left){
			bigarm_angle = (theta_total_ - theta_big_ - BIG_0) / M_TWOPI * REDUCTION_RATIO;
			forearm_angle = (theta_total_ + theta_fore_ - FORE_0)/ M_TWOPI * REDUCTION_RATIO;
		} else {
			bigarm_angle = (theta_total_ + theta_big_ - BIG_1) / M_TWOPI * REDUCTION_RATIO;
			forearm_angle = (theta_total_ - theta_fore_ - FORE_1)/ M_TWOPI * REDUCTION_RATIO;
		}
	}
};







#endif /* INC_IK_H_ */
