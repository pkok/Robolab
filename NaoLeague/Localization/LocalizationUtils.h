#pragma once
class LocalizationUtils
{
public:
	LocalizationUtils(void);
	~LocalizationUtils(void);
	struct Coordinate{
		double x;
		double y;
		double theta;	
	};
	struct Coordinate{
		double x;
		double y;	
	};
	double Eucl_distance(Coordinate, Coordinate);
	double Angle_Difference(double, double);
	double Find_angle_avg(double, double);
	Coordinate Add_coordinates(Coordinate, Coordinate);
	Coordinate Get_symmetric(Coordinate);
	double Find_angle_bet_points(Coordinate, Coordinate);
	Coordinate Find_object_position(Coordinate, double, double);
	Coordinate trilateration(double, double, double,double, double, double);
private:
	bool isnan(double var);
};

