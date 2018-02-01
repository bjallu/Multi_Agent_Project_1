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
	float TimeStep;
	float Velocity;
	float VehicleLength;
	float MaxTurnSpeed;
	FVector GoalVelocity;

	NodeSelector();
	~NodeSelector();
	void GetRrtPath(TArray<FVector>& vectors);
	void RandomPosition(float&, float&);
	void GetPath(TArray<FVector>&);
	void rrt(FVector, FVector);
	FVector CalculatePoint(const FVector&, const FVector&);
	float PointDistance(const FVector&, const FVector&);
	void differentialRrt(const FVector, const FVector, float);
	Node* CalculateDifferentialPoint(const Node& , const FVector&);
	float DifferentialDriveDistance(const Node&, const FVector&);
	float GetGoalOrientation(const FVector &, const FVector&);


};
