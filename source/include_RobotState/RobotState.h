/*____________________________________________________________________
* Copyright (c) ASAMA-YAMASHITA Lab., Dept. of Precision Eng., The Univ. of Tokyo. 
* All rights reserved 2013                                                                 
* Description : Robot state class [ADT]
* Author : Yonghoon JI (ji@robot.t.u-tokyo.ac.jp, ji.nineppl@gmail.com) 
* Date : April, 2013              
* Precondition  :
______________________________________________________________________*/

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#define MM2M 0.001
#define M2MM 1000
#define D2R 0.017453288888889
#define R2D 57.29579143313326

//class __declspec(dllexport) CRobotState
class CRobotState

{
public:



	CRobotState(void);
	~CRobotState(void);

	CRobotState(double x, double y, double z, double roll, double pitch, double yaw)
	{
		initialize();
		this->m_RobotPosition.x = x;
		this->m_RobotPosition.y = y;
		this->m_RobotPosition.z = z;

		this->m_RobotOrientation.roll = roll;
		this->m_RobotOrientation.pitch = pitch;
		this->m_RobotOrientation.yaw = yaw;
	};
// 
// 	CRobotState(CRobotState::Position Posi, CRobotState::Orientation Ori)
// 	{
// 		m_RobotPosition = Posi;
// 		m_RobotOrientation = Ori;
// 
// 	};

	//Unit [m]
	typedef struct _tagPosition
	{
		double x;
		double y;
		double z;
	} Position;

	//Unit [Rad]
	typedef struct _tagOrientation
	{
		double roll;
		double pitch;
		double yaw;
	} Orientation;

	//Unit [m/s]  [Rad/s] 
	typedef struct _tagVelocity
	{
		double v;
		double w;
		double vx;
		double vy;
	} Velocity;

private:

	Position m_RobotPosition;
	Orientation m_RobotOrientation;
	Velocity m_RobotVelocity;

	//Eigen::MatrixXd  m_Covariance;
	int m_nCovDim;
	int m_nID;
	double ** m_Cov;

public:
	inline void setPose(double x, double y, double z, double roll, double pitch, double yaw)
	{
		this->m_RobotPosition.x = x; this->m_RobotPosition.y = y; this->m_RobotPosition.z = z;
			this->m_RobotOrientation.roll = roll;
		if(this->m_RobotOrientation.roll  > 180.0*D2R) this->m_RobotOrientation.roll  -=360.0*D2R;
		if(this->m_RobotOrientation.roll  < -180.0*D2R) this->m_RobotOrientation.roll  +=360.0*D2R;
			this->m_RobotOrientation.pitch = pitch;
		if(this->m_RobotOrientation.pitch  > 180.0*D2R) this->m_RobotOrientation.pitch  -=360.0*D2R;
		if(this->m_RobotOrientation.pitch  < -180.0*D2R) this->m_RobotOrientation.pitch  +=360.0*D2R;
				this->m_RobotOrientation.yaw =yaw;
		if(this->m_RobotOrientation.yaw  > 180.0*D2R) this->m_RobotOrientation.yaw  -=360.0*D2R;
		if(this->m_RobotOrientation.yaw  < -180.0*D2R) this->m_RobotOrientation.yaw  +=360.0*D2R;
	};
	inline void setX(double nX){	this->m_RobotPosition.x = nX;	};
	inline double getX(){	return this->m_RobotPosition.x;};
	inline void setY(double nY)	{	this->m_RobotPosition.y = nY;	};
	inline double getY() { return this->m_RobotPosition.y; };
	inline void setZ(double nZ){	this->m_RobotPosition.z = nZ;	};
	inline double getZ()	{	return this->m_RobotPosition.z;	};
	inline void setRoll(double dRoll)
	{
		this->m_RobotOrientation.roll = dRoll;
		if(this->m_RobotOrientation.roll  > 180.0*D2R) this->m_RobotOrientation.roll  -=360.0*D2R;
		if(this->m_RobotOrientation.roll  < -180.0*D2R) this->m_RobotOrientation.roll  +=360.0*D2R;
	};
	inline double getRoll(){		return this->m_RobotOrientation.roll;	};
	inline void setPitch(double dPitch)
	{
		this->m_RobotOrientation.pitch = dPitch;
		if(this->m_RobotOrientation.pitch  > 180.0*D2R) this->m_RobotOrientation.pitch  -=360.0*D2R;
		if(this->m_RobotOrientation.pitch  < -180.0*D2R) this->m_RobotOrientation.pitch  +=360.0*D2R;
	};
	inline double getPitch(){		return this->m_RobotOrientation.pitch;	};
	inline void setYaw(double dYaw)
	{	
		this->m_RobotOrientation.yaw = dYaw;
		if(this->m_RobotOrientation.yaw  > 180.0*D2R) this->m_RobotOrientation.yaw  -=360.0*D2R;
		if(this->m_RobotOrientation.yaw  < -180.0*D2R) this->m_RobotOrientation.yaw  +=360.0*D2R;
	};
inline double getYaw(){	return this->m_RobotOrientation.yaw;};

inline double getV(){return this->m_RobotVelocity.v;};
inline void setV(double dV) {this,m_RobotVelocity.v = dV; };

inline double getW(){return this->m_RobotVelocity.w;};
inline void setW(double dW) {this,m_RobotVelocity.v = dW; };

inline double getVx(){return this->m_RobotVelocity.vx;};
inline void setVx(double dVx) {this,m_RobotVelocity.vx = dVx; };

inline double getVy(){return this->m_RobotVelocity.vy;};
inline void setVy(double dVy) {this,m_RobotVelocity.vy = dVy; };

inline void setCovarianceDimension(int nCovDim)	
{		
	m_nCovDim = nCovDim;
	m_Cov = new double*[nCovDim];
	for(int i = 0 ; i < nCovDim ; i++)
		m_Cov[i]  = new double[nCovDim];
};

inline void setCovariance(double ** Cov)
{	
	for(int i = 0 ; i < m_nCovDim ; i++)
		for(int j = 0 ; j < m_nCovDim ; j++)
			m_Cov[i][j] = Cov[i][j];
};

inline double ** getCovariance() { return m_Cov; };
inline int getCovarianceDim(){return m_nCovDim;};
inline void setCovarianceElement(int i, int j, double dValue){	m_Cov[i][j] = dValue;};
inline double getCovarianceElement(int i, int j) { return m_Cov[i][j]; };

inline int getID(){return m_nID;};
inline void setID(int nID){m_nID = nID;};


inline void setPosition(CRobotState::Position NewPosition) {	m_RobotPosition = NewPosition;};
inline void setOrientation(CRobotState::Orientation NewOrientation) {	m_RobotOrientation = NewOrientation;};
inline void setVelocity(CRobotState::Velocity NewVelocity) {	m_RobotVelocity = NewVelocity;};
inline CRobotState::Position getPosition(){	return this->m_RobotPosition;};
inline CRobotState::Orientation getOrientation(){	return this->m_RobotOrientation;};
inline CRobotState::Velocity getVelocity(){	return this->m_RobotVelocity;};

inline CRobotState operator+(const CRobotState& another)
{
	CRobotState TempState;

	TempState.setX( m_RobotPosition.x + another.m_RobotPosition.x);
	TempState.setY( m_RobotPosition.y + another.m_RobotPosition.y);
	TempState.setZ( m_RobotPosition.z + another.m_RobotPosition.z);
	TempState.setRoll(m_RobotOrientation.roll + another.m_RobotOrientation.roll);
	TempState.setPitch(m_RobotOrientation.pitch + another.m_RobotOrientation.pitch);
	TempState.setYaw(m_RobotOrientation.yaw + another.m_RobotOrientation.yaw);
	return TempState;
}

inline bool operator==(const CRobotState& another)
{

	if(  m_RobotPosition.x != another.m_RobotPosition.x  || 
		  m_RobotPosition.y != another.m_RobotPosition.y  || 
		  m_RobotPosition.z != another.m_RobotPosition.z  || 
		  m_RobotOrientation.roll != another.m_RobotOrientation.roll ||
		  m_RobotOrientation.pitch != another.m_RobotOrientation.pitch ||
		  m_RobotOrientation.yaw != another.m_RobotOrientation.yaw )
		  return false;
	else 
		return true;
}

inline void initialize()
{
	m_nCovDim = 0;
	m_RobotPosition.x = 0.0;
	m_RobotPosition.y = 0.0;
	m_RobotPosition.z = 0.0;
	m_RobotOrientation.roll = 0.0;
	m_RobotOrientation.pitch = 0.0;
	m_RobotOrientation.yaw = 0.0;
	m_RobotVelocity.v = 0.0;
	m_RobotVelocity.w = 0.0;
	m_RobotVelocity.vx = 0.0;
	m_RobotVelocity.vy = 0.0;
};


};
#endif
