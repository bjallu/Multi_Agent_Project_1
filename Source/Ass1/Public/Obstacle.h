// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "math.h"
#include "Node.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include <vector>

/**
 * 
 */
class ASS1_API Obstacle
{
public:
	int N;
	std::vector<std::vector<int>> points;

	void AddObstaclePoint(float, float);
	Obstacle();
	~Obstacle();
	
};
