#pragma once

#include "PointcloudBasicProcess.h"

using namespace std;

class CHandRegistration : public CPointcloudBasicProcess
{
public:
	void mainProcess();
	void FreeSpace();
private:
	enum KEYNUM {
		NONE,
		LALT,
		X_,
		Y_,
		Z_,
		ROLL_,
		PITCH_,
		YAW_,
		X_MINUS,
		Y_MINUS,
		Z_MINUS,
		ROLL_MINUS,
		PITCH_MINUS,
		YAW_MINUS,
		ENTER,
		RSHIFT,
		RCTRL,
		ESC
	};

	//should declare under enum type declaration
	KEYNUM getKEYNUM();

	void  HandRegistration(string dir_);

	bool M_b_mode_minus;
};
