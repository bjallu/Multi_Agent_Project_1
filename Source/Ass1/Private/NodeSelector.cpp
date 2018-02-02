// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"


NodeSelector::NodeSelector()
{
	XBound = 30.f;
	YBound = 30.f;
	PathSize = 3;
	NumNodes = 1000;
	GoalRadius = 1.f;
	nodes = TArray<Node*>();
	obstacles = TArray<Obstacle>();
	StepSize = 20;
	TimeStep = 0.1;
	VehicleLength = 2;
	Velocity = 1.1;
	MaxTurnSpeed = 1.1;
	GoalVelocity = FVector(0.5f, -0.5f, 0.0f);
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
	y = FMath::RandRange(-YBound, YBound);
}

float NodeSelector::PointDistance(const FVector &p1, const FVector &p2) {
	return sqrt(pow(p1.X - p2.X, 2) + pow(p1.Y - p2.Y, 2));
}

float NodeSelector::GetGoalOrientation(const FVector &goalVel, const FVector& coords) {
	return atan2(goalVel.Y - coords.Y, goalVel.X - coords.X);
}

FVector NodeSelector::CalculatePoint(const FVector &p1, const FVector &p2) {
	if (PointDistance(p1, p2) <= Velocity * TimeStep) {
		return p2; //p2 is the new random point, they are close to eachother
	}
	float theta = atan2(p2.Y - p1.Y, p2.X - p1.X);
	return FVector(p1.X + Velocity * TimeStep * cos(theta), p1.Y + Velocity * TimeStep * sin(theta), 0.f);
}

bool NodeSelector::CollisionCheck(const FVector& pointToCheck, const Obstacle& obstacle) { //Add obstacle class
	int i, j, c = 0;
	for (i = 0, j = obstacle.N - 1; i < obstacle.N; j = i++) {
		if (((obstacle.points[1][i]>pointToCheck.Y) != (obstacle.points[1][j]>pointToCheck.Y)) &&
			(pointToCheck.X < (obstacle.points[0][j] - obstacle.points[0][i]) * (pointToCheck.Y - obstacle.points[1][i]) / (obstacle.points[1][j] - obstacle.points[1][i]) + obstacle.points[0][i]))
			c = !c;
	}
	if (c == 1)
		return true;
	return false;
}

bool NodeSelector::Collides(const FVector& pointToCheck) {
	for (int i = 0; i < obstacles.Num(); ++i) {
		if (CollisionCheck(pointToCheck, obstacles[i])) {
			return false;
		}
	}
	return true;
}

float NodeSelector::DifferentialDriveDistance(const Node& n1, const FVector&n2) {
	//Calculate angle to turn
	float Vx = cos(n1.orientation);
	float Vy = sin(n1.orientation);
	float dot = Vx * Vy + n2.X*n2.Y;
	float det = Vx * n2.Y - n2.X*Vy;
	float angleDistance = atan2(det, dot);
	float turningTime = abs(angleDistance) / MaxTurnSpeed;	
	float distance = PointDistance(n1.point, n2);
	return turningTime + (distance / Velocity);
}

//Always turns and moves forward in maximum speed
Node* NodeSelector::CalculateDifferentialPoint(const Node& n1, const FVector& n2) {
	//Calculate angle to turn
	float Vx = cos(n1.orientation);
	float Vy = sin(n1.orientation);
	float dot = Vx * Vy + n2.X*n2.Y;
	float det = Vx * n2.Y - n2.X*Vy;
	float angleDistance = atan2(det, dot);
	float newOrientation = atan2(n2.Y - n1.point.Y, n2.X - n1.point.X);				
	float turningTime = abs(angleDistance) / MaxTurnSpeed;		// FEL
	//Maybe add a time variable to nodes so that we can simulate the turning on a node of its own.
	if (TimeStep - turningTime < 0) {
		float orientation = n1.orientation + MaxTurnSpeed * TimeStep;
		return new Node(n1, n1.point, orientation);
	}

	float TimeLeft = TimeStep - turningTime;
	float x = n1.point.X + Velocity * cos(n1.orientation+angleDistance)*(TimeLeft);
	float y = n1.point.Y + Velocity * sin(n1.orientation + angleDistance)*(TimeLeft);
	Node * node = new Node(n1, FVector(x, y, n1.point.Z), n1.orientation + angleDistance);
	return node;
}

// Should work for every model except the car
bool NodeSelector::CheckTrivialPath(const FVector& from, const FVector &to) {
	TArray<FVector> trivialPath = TArray<FVector>();
	if (PointDistance(from, to) <= Velocity * TimeStep) {
		return true;
	}
	float theta = atan2(to.Y - from.Y, to.X - from.X);
	FVector newNode = FVector(from.X + Velocity * TimeStep * cos(theta), from.Y + Velocity * TimeStep * sin(theta), 0.f);
	while (!Collides(newNode)) {
		if (PointDistance(newNode, to) <= Velocity * TimeStep) {
			trivialPath.Add(newNode);
			trivialPath.Add(to);
			//Add trivialPath to NodePath

			return true;
		}
		trivialPath.Add(newNode);
		newNode = FVector(newNode.X + Velocity * TimeStep * cos(theta), newNode.Y + Velocity * TimeStep * sin(theta), 0.f);
	}
	return false;
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
		//Check if there is a straight line to the target from the current position

		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(-XBound, XBound);
			rand.Y = FMath::RandRange(-YBound, YBound);
			for (int i = 0; i < nodes.Num(); ++i) {
				if (PointDistance(nodes[i]->point, rand) <= PointDistance(parent->point, rand)) {
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
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("%d"), nodes.Num());
}

void NodeSelector::differentialRrt(const FVector EndPosition, const FVector StartPosition, float startOrientation, float EndOrientation) {
	nodes.Empty();
	//Create startnode
	Node* StartNode = new Node(StartPosition, startOrientation);
	nodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = EndPosition.X;
	float y = EndPosition.Y;
	FVector rand = FVector(x, y, StartPosition.Z);
	Node* parent = nodes[0];
	Node* NewNode = parent;
	while (count < NumNodes) {
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(-XBound, XBound);
			rand.Y = FMath::RandRange(-YBound, YBound);
			if (count % 20 == 0) {
				rand.X = EndPosition.X;
				rand.Y = EndPosition.Y;
				
				//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
			}
			for (int i = 0; i < nodes.Num(); ++i) {
				float returned = DifferentialDriveDistance(*nodes[i], rand);
				float returned2 = DifferentialDriveDistance(*parent, rand);
				//float returned = PointDistance(nodes[i]->point, rand);
				//float returned2 = PointDistance(parent->point, rand);
				if (returned <= returned2) {
					NewNode = CalculateDifferentialPoint(*nodes[i], rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		NewNode->point.Z = 70.f;
		nodes.Add(new Node(parent, NewNode->point, NewNode->orientation));
		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition, NewNode->orientation));
			float goalOrientation = GetGoalOrientation(GoalVelocity, EndPosition);
			Node* previous = NewNode;
			//Rotate to the correct position
			while (abs(goalOrientation - NewNode->orientation) < 0.1f) {
				//Rotate into place
				previous = NewNode;
				//Just rotate I cant think cuz its 03.40 so ill fix later
				nodes.Add(new Node(previous, NewNode->point, NewNode->orientation));
			}
				
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
