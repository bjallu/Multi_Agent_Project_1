// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"


NodeSelector::NodeSelector()
{
	XBound = 300.f;
	YBound = 300.f;
	PathSize = 3;
	NumNodes = 200;
	GoalRadius = 0.1f;
	nodes = TArray<Node*>();
	StepSize = 20;
}

NodeSelector::~NodeSelector()
{
}

//run rrt first to create the nodes
void NodeSelector::GetRrtPath(TArray<FVector>& vectors) {
	//nodes[0] is startposition so shouldnt include it
	Node* CurrentNode = nodes[nodes.Num() - 1];
	while (CurrentNode->point != nodes[0]->point) {
		CurrentNode->point.Z = 0;
		vectors.Add(CurrentNode->point);
		CurrentNode = CurrentNode->parent;
	}
	Algo::Reverse(vectors);
}

void NodeSelector::RandomPosition(float& x, float& y) {
	x = FMath::RandRange(-XBound, XBound);
	y = FMath::RandRange(-YBound,YBound);
}

float NodeSelector::PointDistance(const FVector &p1, const FVector &p2) {
	return (p1 - p2).Size();
}

FVector NodeSelector::CalculatePoint(const FVector &p1,const FVector &p2) {
	if (PointDistance(p1,p2)<=StepSize){
		return p2; //p2 is the new random point, they are close to eachother
	}
	float theta = atan2(p2.Y - p1.Y, p2.X - p1.X);
	return FVector(p1.X + StepSize * cos(theta), p1.Y + StepSize * sin(theta), 0.f);
}

void NodeSelector::rrt(FVector EndPosition, FVector StartPosition) {
	nodes.Empty();
	//Create startnode
	Node* StartNode = new Node(StartPosition);
	nodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	Node* parent = nodes[0];
	while (count < NumNodes) {
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(-XBound, XBound);
			rand.Y = FMath::RandRange(-YBound, YBound);
			for (int i = 0; i < nodes.Num(); ++i) {
				if (PointDistance(nodes[i]->point, rand) <= PointDistance(parent->point,rand)) {
					FVector NewPoint = CalculatePoint(nodes[i]->point, rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		FVector NewNode = CalculatePoint(parent->point, rand);
		NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent.point.X, parent.point.Y);

		nodes.Add(new Node(parent, NewNode));

		if (PointDistance(NewNode, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition));
			count = NumNodes;
		}

		
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("%d"), nodes.Num());

	
}

void NodeSelector::GetPath(TArray<FVector>& vectors)
{
	float x;
	float y;
	for (int i = 0; i < PathSize; ++i) {
		x = FMath::RandRange(-XBound, XBound);
		y = FMath::RandRange(-YBound, YBound);
		vectors.Add(FVector(x, y, 0.f));
		//UE_LOG(LogTemp, Display, TEXT("Goal X: %f Goal Y: %f"), x, y);
	}
}
