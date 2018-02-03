// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "math.h"
#include "Node.h"
#include "Algo/Reverse.h"
#include "Obstacle.h"
#include "DrawDebugHelpers.h"
#include "DynamicNode.h"
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
	float Acceleration;
	TArray<Node*> nodes;
	TArray<DynamicNode*> DynamicNodes;
	TArray<Obstacle> obstacles;
	float TimeStep;
	float Velocity;
	float VehicleLength;
	float MaxTurnSpeed;
	FVector GoalVelocity;

	NodeSelector();
	~NodeSelector();
	void GetRrtPath(TArray<Node*>& vectors);
	void RandomPosition(float&, float&);
	void rrt(FVector, FVector);
	FVector CalculatePoint(const FVector&, const FVector&);
	float PointDistance(const FVector&, const FVector&);
	void differentialRrt(const FVector, const FVector, const FVector, const FVector);
	Node* CalculateDifferentialPoint(const Node& , const FVector&);
	float DifferentialDriveDistance(const Node&, const FVector&);
	float GetGoalOrientation(const FVector &, const FVector&);
	float GetCosAngle(const FVector&, const FVector&);
	void dynamicPointRrt(FVector, FVector, FVector, FVector);
	void GetDynamicRrtPath(TArray<DynamicNode*>&);
	DynamicNode* CalculateDynamicPointNode(const DynamicNode&, FVector);



};
