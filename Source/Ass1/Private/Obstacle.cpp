// Fill out your copyright notice in the Description page of Project Settings.
#include "Obstacle.h"


Obstacle::Obstacle()
{
	double lowest(std::numeric_limits<double>::lowest());
	double max(std::numeric_limits<double>::max());

	this->N = 0;
	//this->points = vector<vector<int>>(10);
	this->maxY = lowest;
	this->minY = max;
	this->maxX = lowest;
	this->minX = max;

	//double infinity(std::numeric_limits<double>::infinity());
	//double neg_infinity(-std::numeric_limits<double>::infinity());
	//double lowest(std::numeric_limits<double>::lowest());

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