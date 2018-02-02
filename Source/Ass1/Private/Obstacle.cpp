// Fill out your copyright notice in the Description page of Project Settings.
#include "Obstacle.h"


Obstacle::Obstacle()
{
	this->N = 0;
	//this->points = vector<vector<int>>(10);
}

Obstacle::~Obstacle()
{
}

void Obstacle::AddObstaclePoint(float x, float y) {
	std::vector<int> np(2);
	np.push_back(x);
	np.push_back(y);
	this->points.push_back(np);
}

/*
class Line {
public:
	void setLength(double len);
	double getLength(void);
	Line(double len);  // This is the constructor

private:
	double length;
};

// Member functions definitions including constructor
Line::Line(double len) {
	cout << "Object is being created, length = " << len << endl;
	length = len;
}
void Line::setLength(double len) {
	length = len;
}
double Line::getLength(void) {
	return length;
}
*/