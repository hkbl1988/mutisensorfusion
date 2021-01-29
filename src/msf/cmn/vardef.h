#ifndef MSF_VARDEF_H_
#define MSF_VARDEF_H_

#include "cmn.h"
#include "../src/process/preprocess.h"

namespace msf {
namespace common {

	const int ADIM = 22;
	const int GDIM = 6;
	const int RDIM = 1;
	const int UDIM = 22;

	const double mStaticTime = 3;		//static time 3s
	const double STATIC_VEL_THESHOLD = 0.005;
	const double MOTION_VEL_THESHOLD = 2;
	const double kSOLTIME = 0.01;

	const double mHDOP = 3.0;	// max hdop limit
	const int mSATENUM = 10;	// min satenum limit

	const double mPOS_ERROR = 0.3;				// max pos error 30cm
	const double mATT_ERROR = 2 * kDEG2RAD;		// max att error 2deg 
	const double mVEL_ERROR = 2;				// max vel error 2m/s

	const double MAX_POS_ERROR = 50.0 * mPOS_ERROR;
	const double MAX_ATT_ERROR = 50.0 * mATT_ERROR;

	const double   fIMU_FREQ = 50;
	const double  fGNSS_FREQ = 10;
	const double  fODOM_FREQ = 10;
	const double fLIDAR_FREQ = 3;

	const double   tIMU_SPAN = 1.0 / fIMU_FREQ;
	const double  tGNSS_SPAN = 1.0 / fGNSS_FREQ;
	const double  tODOM_SPAN = 1.0 / fODOM_FREQ;
	const double tLIDAR_SPAN = 1.0 / fLIDAR_FREQ;

	const int  kIMU_GNSS = fIMU_FREQ / fGNSS_FREQ;
	const int  kIMU_ODOM = fIMU_FREQ / fODOM_FREQ;
	const int kIMU_LIDAR = fIMU_FREQ / fLIDAR_FREQ;

	const double MAX_GBIAS = 1000.0 / 3600.0;		//max gyro bias is 350deg/h
	const double MAX_ABIAS = 200.0 * kM_GRAVITY;	//max acce bias is 20mg

	struct output_elements{
		
		std_msgs::Header header;
		double deltime_ros;
		double drtime;
		Vector3d angular;
		Vector3d acceler;
		Vector3d attitude;
		Vector3d velocity;
		Vector3d position;
		Vector3d position_inf;
		Vector3d velodom;
		Vector4i lidarcnt;
		Vector7i gnsscnt;
		Vector3i odomcnt;
		Vector3d gbias_init;
		STATUS workmode;
		Vector3d abias;
		Vector3d gbias;
		Vector3d states;
		Vector3d posoffset;
		double yawoffset;
		double speed;
		Vector3d odom_error;
		bool odom_error_flag;
		STATUS vehicle_status;
		double gyro_var_norm;
		double acce_var_norm;

		output_elements(){
			deltime_ros = 0.0;
			angular.setZero();
			acceler.setZero();
			attitude.setZero();
			velocity.setZero();
			position.setZero();
			position_inf.setZero();
			velodom.setZero();
			drtime = 0;
			lidarcnt.setZero();
			gnsscnt.setZero();
			odomcnt.setZero();
			workmode = STATUS::READY;
			abias.setZero();
			gbias.setZero();
			states.setZero();
			posoffset.setZero();
			yawoffset = 0.0;
			odom_error.setZero();
			odom_error_flag = false;
			vehicle_status = STATUS::STATIC;
			gyro_var_norm = acce_var_norm = 0.0;
		}

		void update_states(Eigen::Matrix<double,ADIM,1> &X)
		{
			states(0) -= X(15) * kRAD2DEG; 				// units:deg
			states(1) -= X(16);					
			states(2) -= X(17) * kRAD2DEG; 				// units:deg
		}
	};

	struct imu_elements{
		
		std_msgs::Header header;
		bool update;
		unsigned int time_ms;
		double deltime_ms;
		double deltime_ros;
		Vector3d angular;
		Vector3d acceler;
		Vector3d gbias;
		Vector3d abias;

		imu_elements(){
			update = false;
			time_ms = 0;
			deltime_ms = deltime_ros = 0.0;
			angular.setZero();
			acceler.setZero();
			gbias.setZero();
			abias.setZero();	
		}

		void GetMsg(const ImuPose &msg)
		{
			header.stamp = msg.header.stamp;
			update = true;
			time_ms = msg.tag;
			angular(0) = msg.gyro_x;  	
			angular(1) = msg.gyro_y;
			angular(2) = msg.gyro_z;
			acceler(0) = msg.acce_x;
			acceler(1) = msg.acce_y;			// unit m/s^2
			acceler(2) = msg.acce_z;

		}
	};

	struct  odom_elements
	{
		std_msgs::Header header;
		bool update;
		bool initial;
		double deltime_ros;
		double angular_l,angular_r;
		double velocity_l,velocity_r;
		double velocity_filter;
		Vector3d velocity_add;
		Vector3d vel_xyz;
		Vector3d velocity;
		Vector3d position;
		Vector3d position_inf;
		double yaw_rate;
		double coeff;
		odom_elements()
		{
			update = false;
			initial = false;
			deltime_ros = 0.0;
			angular_l = angular_r = 0.0;
			velocity_l = velocity_r = 0.0;
			velocity_filter = 0.0;
			yaw_rate = 0.0;
			velocity_add.setZero();
			vel_xyz.setZero();
			velocity.setZero();
			position.setZero();
			position_inf.setZero();
			coeff = 0.75;
		}
		void GetMsg(const OdomPose &msg,Matrix3d cbn)
		{
			header.stamp = msg.header.stamp;
			update = true;
			//get odom velocity
			velocity_l = msg.velocity_rl;
			velocity_r = msg.velocity_rr;
			vel_xyz(1) = msg.velocity;
			velocity = cbn * vel_xyz;
		}
		void SetPos(const Vector3d pos)
		{
			initial = true;
			position = pos;
		}

		void UpdatePos(Vector3d laram,Vector3d angular,Matrix3d cbn,const double tau)
		{
			if(initial) 
			{	
				Vector3d vel_enu;
				vel_enu = cbn * vel_xyz;
				position += (vel_enu + cbn * Cross(angular,laram)) * tau;
				position_inf += vel_enu * tau;
			}
		}
		void CompeArm(Vector3d laram,Vector3d angular,Matrix3d cbn)
		{
			velocity_add = cbn * Cross(angular,laram);
			velocity += velocity_add;
		}
	};


	struct gnss_pos_elements
	{
		std_msgs::Header header;
		bool update;
		bool initial;
		double deltime_ros;
		double lat,lon,alt;
		Vector3d position;
		Vector3d position_add;
		Vector3d position_init;
		Vector3d velocity;
		Vector3d pos_std;
		Vector3d vel_std;
		Vector3d vel_xyz;
		int status;
		int satenum;
		double hdop;
		double speed;
		gnss_pos_elements()
		{
			update = false;
			initial = false;
			deltime_ros = 0.0;
			lat = 40;
			lon = 116;
			alt = 43;
			position.setZero();
			position_add.setZero();
			position_init.setZero();
			velocity.setZero();
			pos_std.setZero();
			vel_std.setZero();
			vel_xyz.setZero();
			status = 0;
			satenum = 0;
			hdop = 0;
			speed = 0;
		}
		void GetMsg(const GnssPose &msg,Matrix3d cbn,unsigned char vehicle_status)
		{
			header.stamp = msg.header.stamp;
			if(vehicle_status == STATUS::BACKWARD){
				vel_xyz(1) = -msg.vel;
			}else{
				vel_xyz(1) = msg.vel;
			}
			lat = msg.lat;
			lon = msg.lon;
			alt = msg.alt;
			status = msg.status;
			satenum = msg.satenum;
			hdop = msg.hdop;
			position(0) = msg.xg;
			position(1) = msg.yg;
			position(2) = msg.zg;
			velocity = cbn * vel_xyz;
			speed = msg.vel;
		}
		void CompeArm(Vector3d laram,Matrix3d cbn)
		{
			position_add = cbn * laram;
			position += position_add;
		}
	};


	struct  gnss_yaw_elements
	{
		std_msgs::Header header;
		bool update;
		bool initial;
		double deltime_ros;
		double yaw;
		double yaw_std;
		int satenum;
		int status_yaw;
		double hdop;
		double base_length;
		double yaw_init;
		gnss_yaw_elements()
		{
			update = false;
			initial = false;
			deltime_ros = 0.0;
			yaw = 0.0;
			yaw_std = 0.0;
			satenum = 0;
			hdop = 0;
			base_length = 0.0;
			yaw_init = 0.0;
		}
		void GetMsg(const GnssPose &msg,double dyaw)
		{
			header.stamp = msg.header.stamp;
		//	update = true;
			yaw = msg.yaw * kDEG2RAD;
			satenum = msg.satenum;
			status_yaw = msg.yawstatus;
			hdop = msg.hdop;
			base_length = msg.baselen;
		}
	};


	struct  lidar_elements
	{
		std_msgs::Header header;
		bool update;
		double deltime_ros;
		double lat,lon,alt;
		double roll;
		double pitch;
		double yaw;
		double trans_probability;
		Vector3d position;
		Vector3d position_add;
		Vector3d pos_std;
		lidar_elements()
		{
			update = false;
			deltime_ros = 0.0;
			lat = 40;
			lon = 116;
			alt = 43;
			roll = pitch = yaw = 0.0;
			trans_probability = 0.0;
			position.setZero();
			position_add.setZero();
			pos_std.setZero();
		}
		void GetMsg(const LidarPose &msg)
		{
			header.stamp = msg.header.stamp;
			update = true;
			roll = msg.roll;
			pitch = msg.pitch;
			yaw = msg.yaw * kDEG2RAD;
			position(0) = msg.xg;
			position(1) = msg.yg;
			position(2) = msg.zg;
			trans_probability = msg.trans_probability;
		}
		void CompeArm(Vector3d laram,Matrix3d cbn)
		{
			position_add = cbn * laram;
			position += position_add;
		}
	};

	struct imu_variance_elements
	{	
		double time;
		Vector3d gyro_mean;
		Vector3d acce_mean;
		Vector3d gyro_var;
		Vector3d acce_var;
		double gyro_var_norm;
		double acce_var_norm;
		imu_variance_elements()
		{
			gyro_mean(100);
			acce_mean(100);
			gyro_var(100);
			acce_var(100);
			gyro_var_norm = 100;
			acce_var_norm = 100;
		}	

		void GetMsg(std::deque<imu_elements> imu_deque)
		{	
			Vector3d accum_gyro;accum_gyro.setZero();
			Vector3d accum_acce;accum_acce.setZero();
			double accum_time = 0.0;
			int len = imu_deque.size();
			if(len < 10) return;
			for(int k = 0; k < len; k++)
			{
				accum_time += imu_deque[k].header.stamp.toSec();
				accum_gyro += imu_deque[k].angular;
				accum_acce += imu_deque[k].acceler;
			}

			time = accum_time / len;
			gyro_mean = accum_gyro / len;
			acce_mean = accum_acce / len;

			gyro_var.setZero();
			acce_var.setZero();

			for(int k = 0; k < len; k++)
			{
				Vector3d gyro_dev = imu_deque[k].angular - gyro_mean;
				Vector3d acce_dev = imu_deque[k].acceler - acce_mean;

				for(int i = 0; i < 3; i++)
				{
					gyro_var(i) += gyro_dev(i) * gyro_dev(i);
					acce_var(i) += acce_dev(i) * acce_dev(i);
				}
			}

			gyro_var /= (len - 1);
			acce_var /= (len - 1);

			gyro_var_norm = gyro_var.norm();
			acce_var_norm = acce_var.norm();
		}
	};

	struct  dpos_elements
	{
		int Step;
		int maxStep;
		Vector3d dpos;
		dpos_elements()
		{
			Step = 1;
			maxStep = -1;
			dpos.setZero();
		}
		void Initial()
		{
			Step = 1;
			maxStep = -1;
			dpos.setZero();
		}
		void GetMsg(const Vector3d pos,int mStep)
		{
			Step = 1;
			maxStep = mStep;
			if(maxStep == 0) maxStep = 1;
			double f = 1.0 / maxStep;
			dpos.setZero();
			if(sqrt(pos(0) * pos(0) + pos(1) * pos(1)) < mPOS_ERROR)
			{
				dpos(0) = pos(0) * f;
				dpos(1) = pos(1) * f;
			}
			if(fabs(pos(2)) <= mPOS_ERROR) dpos(2) = pos(2) * f;
		}
	};

	struct dang_elements
	{
		int Step;
		int maxStep;
		Vector3d dang;
		dang_elements()
		{
			Step = 1;
			maxStep = -1;
			dang.setZero();
		}
		void Initial()
		{
			Step = 1;
			maxStep = -1;
			dang.setZero();
		}
		void GetMsg(const Vector3d ang,int mStep)
		{
			Step = 1;
			maxStep = mStep;
			if(maxStep == 0) maxStep = 1;
			double f = 1.0 / maxStep;
			dang.setZero();
			if(fabs(ang(0)) <= mATT_ERROR) dang(0) = ang(0) * f;
			if(fabs(ang(1)) <= mATT_ERROR) dang(1) = ang(1) * f;
			if(fabs(ang(2)) <= mATT_ERROR) dang(2) = ang(2) * f;
		}
	};

	struct average_elements
	{
		int cnt;
		int Status;
		double Time;
		double maxTime;
		Vector3d src_1;
		Vector3d src_2;
		Vector3d avg_tmp_1;
		Vector3d avg_tmp_2;
		Vector3d avg1;
		Vector3d avg2;
		average_elements()
		{
			cnt = 0;
			Status = 0;
			Time = 0.0;
			maxTime = mStaticTime;
			avg_tmp_1.setZero();
			avg_tmp_2.setZero();
			avg1.setZero();
			avg2.setZero();
		}

		void InitAvg(double mTime)
		{
			cnt = 0;
			Status = 0;
			Time = 0.0;
			maxTime = mTime;
			src_1.setZero();
			src_2.setZero();
			avg_tmp_1.setZero();
			avg_tmp_2.setZero();
			avg1.setZero();
			avg2.setZero();
		}

		int GetAvg(Vector3d src1,Vector3d src2,double tau)
		{
			cnt++;
			Time += tau;

			if(1 == cnt)
			{
				src_1 = src1;
				src_2 = src2;
			}

			avg_tmp_1 += src1;
			avg_tmp_2 += src2;

			if(Time >= maxTime)
			{
				src_1.setZero();
				src_2.setZero();
				avg1 = avg_tmp_1 / cnt;
				avg2 = avg_tmp_2 / cnt;
				cnt = 0;
				Time = 0;
				avg_tmp_1.setZero();
				avg_tmp_2.setZero();
				if(Status!=2) Status = 1;
			}
			return Status;
		}
	};

	struct feedback_elements
	{
		int Step;
		int maxStep;
		int integral_yaw;
		int integral_pos[3];
		int integral_vel[3];
		Vector3d attError;
		Vector3d posError;
		Vector3d velError;
		feedback_elements()
		{
			Step = 1;
			maxStep = -1;
			integral_yaw = 5000;
			integral_pos[0] = integral_pos[1] = integral_pos[2] = 1000;
			integral_vel[0] = integral_vel[1] = integral_vel[2] = 1000;
			attError.setZero();
			posError.setZero();
			velError.setZero();
		}
		void Initial()
		{
			Step = 1;
			maxStep = -1;
			integral_yaw = 5000;
			integral_pos[0] = integral_pos[1] = integral_pos[2] = 1000;
			integral_vel[0] = integral_vel[1] = integral_vel[2] = 1000;
			attError.setZero();
			posError.setZero();
			velError.setZero();
		}
		void GetMsg(Eigen::Matrix<double,ADIM,1> &X,int mStep)
		{
			Step = 1;
			maxStep = mStep;
			if(maxStep == 0) maxStep = 1;
			double f = 1.0 / maxStep;
			attError.setZero();
			posError.setZero();
			velError.setZero();
			if(sqrt(X(0) * X(0) + X(1) * X(1)) <= mATT_ERROR) 
			{
				attError(0) = X(0) * f;
				attError(1) = X(1) * f;
			}
			if(fabs(X(2)) <= mATT_ERROR) attError(2) = X(2) * f;
			if(fabs(X(3)) <= mVEL_ERROR) velError(0) = X(3) * f;
			if(fabs(X(4)) <= mVEL_ERROR) velError(1) = X(4) * f;
			if(fabs(X(5)) <= mVEL_ERROR) velError(2) = X(5) * f;
			if(fabs(X(6)) <= mPOS_ERROR) posError(0) = X(6) * f;
			if(fabs(X(7)) <= mPOS_ERROR) posError(1) = X(7) * f;
			if(fabs(X(8)) <= mPOS_ERROR) posError(2) = X(8) * f;
		}
	};


	struct covariance_elements
	{
		Vector3d attitude;
		Vector3d velocity;
		Vector3d position;
		Vector3d gyrobias;
		Vector3d accebias;
		double yaw_error;
		double k_factor;
		double theta_error;
		Vector3d posoffset;
		double yawoffset;
		covariance_elements()
		{
			attitude.setZero();
			velocity.setZero();
			position.setZero();
			gyrobias.setZero();
			accebias.setZero();
			yaw_error = 0.0;
			k_factor = 0.0;
			theta_error = 0.0;
			posoffset.setZero();
			yawoffset = 0.0;
		}
		void GetMsg(Eigen::Matrix<double,ADIM,ADIM> &P)
		{
			attitude(0) = sqrt(P(0,0)) * kRAD2DEG;
			attitude(1) = sqrt(P(1,1)) * kRAD2DEG;
			attitude(2) = sqrt(P(2,2)) * kRAD2DEG;
			velocity(0) = sqrt(P(3,3));
			velocity(1) = sqrt(P(4,4));
			velocity(2) = sqrt(P(5,5));
			position(0) = sqrt(P(6,6));
			position(1) = sqrt(P(7,7));
			position(2) = sqrt(P(8,8));
			gyrobias(0) = sqrt(P(9,9)) * kRAD2DEG;
			gyrobias(1) = sqrt(P(10,10)) * kRAD2DEG;
			gyrobias(2) = sqrt(P(11,11)) * kRAD2DEG;
			accebias(0) = sqrt(P(12,12)) / kM_GRAVITY;
			accebias(1) = sqrt(P(13,13)) / kM_GRAVITY;
			accebias(2) = sqrt(P(14,14)) / kM_GRAVITY;
			yaw_error = sqrt(P(15,15)) * kRAD2DEG;
			k_factor = sqrt(P(16,16));
			theta_error = sqrt(P(17,17)) * kRAD2DEG;
			posoffset(0) = sqrt(P(18,18));
			posoffset(1) = sqrt(P(19,19));
			posoffset(2) = sqrt(P(20,20));
			yawoffset = sqrt(P(21,21)) * kRAD2DEG;
		}
	};
}  // namespace common
}  // namespace msf

#endif 