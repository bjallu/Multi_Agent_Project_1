// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "math.h"
#include "Node.h"
#include "Algo/Reverse.h"
#include "Obstacle.h"
#include "DrawDebugHelpers.h"
#include "DynamicNode.h"
#include "CarNode.h"
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
	TArray<CarNode*> CarNodes;
	TArray<Obstacle> obstacles;
	float TimeStep;
	float Velocity;
	float VehicleLength;
	float MaxTurnSpeed;
	float MaxTurnAngle;
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
	void getCarRrtPath(TArray<CarNode*>&);
	DynamicNode* CalculateDynamicPointNode(const DynamicNode&, FVector);
	void carRrt(FVector EndPosition, FVector StartPosition, FVector StartVelocity, FVector EndVelocity);
	CarNode* CalculateCarNode(const CarNode&n1, const FVector n2);
	CarNode* GetDubinsPath(const CarNode&n1, const CarNode&n2);
	TArray<CarNode*> CalculateTangentPoints(CarNode& n1, CarNode& n2);
	std::vector<std::pair<FVector, FVector>>  TangentLines(FVector c1, FVector c2, float radc1, float radc2);
	float ArcLength(FVector center, FVector left, FVector right, float radius, bool isleft);
	TArray<CarNode*> LSR(std::vector<std::pair<FVector, FVector>>& _LRTangents, const FVector agentleft, float radleft, const FVector goalright, float radright, FVector currPos, FVector goalPos,const CarNode&n1);



};
