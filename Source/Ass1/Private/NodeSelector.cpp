// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"


NodeSelector::NodeSelector()
{
	XBound = 30.f;
	YBound = 30.f;
	PathSize = 3;
	NumNodes = 1000;
	GoalRadius = 10.f;
	nodes = TArray<Node*>();
	StepSize = 20;
	TimeStep = 0.1;
	VehicleLength = 2;
	Velocity = 1.1;
	MaxTurnSpeed = 2 * Velocity / VehicleLength;
	GoalVelocity = FVector(0.5f, -0.5f,0.0f);
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
	return sqrt(pow(p1.X-p2.X,2)+pow(p1.Y-p2.Y,2));
}

float NodeSelector::GetGoalOrientation(const FVector &goalVel, const FVector& coords) {
	return atan2(goalVel.Y - coords.Y, goalVel.X - coords.X);
}

FVector NodeSelector::CalculatePoint(const FVector &p1,const FVector &p2) {
	if (PointDistance(p1,p2)<=Velocity*TimeStep){
		return p2; //p2 is the new random point, they are close to eachother
	}
	float theta = atan2(p2.Y - p1.Y, p2.X - p1.X);
	return FVector(p1.X + Velocity*TimeStep * cos(theta), p1.Y + Velocity*TimeStep * sin(theta), 0.f);
}

float NodeSelector::DifferentialDriveDistance(const Node& n1, const FVector&n2) {
	//Calculate angle to turn
	float newOrientation = atan2(n1.point.Y - n2.Y, n1.point.X - n2.X);
	float turningTime = (newOrientation - n1.orientation)*VehicleLength / MaxTurnSpeed;
	float distance = PointDistance(n1.point, n2);
	//UE_LOG(LogTemp, Display, TEXT("Turningtime: %f"), turningTime);
	return turningTime + (distance / Velocity);
}

//Always turns and moves forward in maximum speed
Node* NodeSelector::CalculateDifferentialPoint(const Node& n1, const FVector& n2) {
	//Calculate angle to turn
	float newOrientation = atan2(n1.point.Y - n2.Y, n1.point.X - n2.X);
	float turningTime = (newOrientation - n1.orientation)*VehicleLength / MaxTurnSpeed;
	if (turningTime < 0) {
		UE_LOG(LogTemp, Display, TEXT("TURNING TIME NEGATIVE: %f"), turningTime);
	}
	//Maybe add a time variable to nodes so that we can simulate the turning on a node of its own.
	if (TimeStep - turningTime < 0) {
		float orientation = n1.orientation + MaxTurnSpeed*TimeStep; 
		return new Node(n1, n1.point, orientation);
	}
	float TimeLeft = TimeStep - turningTime;
	//Move forward as much as possible
	float x = n1.point.X + Velocity * cos(newOrientation)*(TimeLeft);
	float y = n1.point.X + Velocity * sin(newOrientation)*(TimeLeft);
	Node * node = new Node(n1, FVector(x, y, 0), newOrientation);
	//UE_LOG(LogTemp, Display, TEXT("Inside function x,y: %f, %f"), n1.point.X, n1.point.Y);
	return node;
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

void NodeSelector::differentialRrt(const FVector EndPosition, const FVector StartPosition, float startOrientation) {
	nodes.Empty();
	//Create startnode
	Node* StartNode = new Node(StartPosition, startOrientation);
	nodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	Node* parent = nodes[0];
	float returned = DifferentialDriveDistance(*parent, rand);
	UE_LOG(LogTemp, Display, TEXT("Distance currNode to random: %f"), returned);
	
	while (count < NumNodes) {
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(-XBound, XBound);
			rand.Y = FMath::RandRange(-YBound, YBound);
			for (int i = 0; i < nodes.Num(); ++i) {
				float returned = DifferentialDriveDistance(*nodes[i], rand);
				float returned2 = DifferentialDriveDistance(*parent, rand);
				if (returned <= returned2) {
					Node* NewPoint = CalculateDifferentialPoint(*nodes[i], rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		Node* NewNode = CalculateDifferentialPoint(*parent, rand); 
		//UE_LOG(LogTemp, Display, TEXT("NewNode pointX and Y: %d, %d"),NewNode->point.X, NewNode->point.Y);
		NewNode->point.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent.point.X, parent.point.Y);

		nodes.Add(new Node(parent, NewNode->point, parent->orientation));
		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition, parent->orientation));
			float goalOrientation = GetGoalOrientation(GoalVelocity, EndPosition);
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		count++;
		//UE_LOG(LogTemp, Display, TEXT("%d"), count);
	}
	//UE_LOG(LogTemp, Display, TEXT("%d"), nodes.Num());
	
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
