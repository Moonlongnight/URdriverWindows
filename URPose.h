#pragma once
#define float64 double
class URPose {
public:
	float64 x;
	float64 y;
	float64 z;
	float64 rx;
	float64 ry;
	float64 rz;
	
	URPose(double X = 0, double Y = 0, double Z=0, double RX = 0, double RY = 0, double RZ = 0) {
		x = X;
		y = Y;
		z = Z;
		rx = RX;
		ry = RY;
		rz = RZ;
	}

	URPose operator =(const URPose& p2) {
		this->x = p2.x;
		this->y = p2.y;
		this->z = p2.z;
		this->rx = p2.rx;
		this->ry = p2.ry;
		this->rz = p2.rz;
		return p2;
	}
};
