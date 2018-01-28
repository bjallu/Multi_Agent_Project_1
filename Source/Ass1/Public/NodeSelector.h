// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "math.h"
#include "Node.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

/**
 * 
 */
class ASS1_API NodeSelector
{
public:
	float XBound;
	float YBound;
	float PathSize;
	float NumNodes;
	float GoalRadius;
	float StepSize;
	TArray<Node*> nodes;

	NodeSelector();
	~NodeSelector();
	void GetRrtPath(TArray<FVector>& vectors);
	void RandomPosition(float&, float&);
	void GetPath(TArray<FVector>&);
	void rrt(FVector, FVector);
	FVector CalculatePoint(const FVector&, const FVector&);
	float PointDistance(const FVector&, const FVector&);

};
