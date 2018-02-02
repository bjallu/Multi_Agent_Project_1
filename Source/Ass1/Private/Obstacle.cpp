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
