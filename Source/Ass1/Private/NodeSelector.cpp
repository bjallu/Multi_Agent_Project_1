// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"


NodeSelector::NodeSelector()
{
	XBound = 30.f;
	YBound = 30.f;
	PathSize = 3;
	NumNodes = 3000;
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
void NodeSelector::GetRrtPath(TArray<Node*>& vectors) {
	//nodes[0] is startposition so shouldnt include it
	Node* CurrentNode = nodes[nodes.Num() - 1];
	while (CurrentNode->point != nodes[0]->point) {
		CurrentNode->point.Z = 0;
		vectors.Add(CurrentNode);
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
	return atan2(goalVel.Y - coords.Y, goalVel.X - coords.X); // is wrong
}

FVector NodeSelector::CalculatePoint(const FVector &p1, const FVector &p2) {
	if (PointDistance(p1, p2) <= Velocity * TimeStep) {
		return p2; //p2 is the new random point, they are close to eachother
	}
	FVector dir = p2 - p1;
	dir.Z = 0;
	dir = dir / dir.Size(); //Normalize
	float theta = atan2(p2.Y - p1.Y, p2.X - p1.X);
	return FVector(p1.X + Velocity * TimeStep * dir.X, p1.Y + Velocity * TimeStep * dir.Y, 0.f);
}

bool NodeSelector::CollisionCheck(const FVector& pointToCheck, const Obstacle& obstacle) {
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
	FVector path = n2 - n1.point;
	float angleDistance = acos(GetCosAngle(n1.orientation, path));
	float turningTime = angleDistance / MaxTurnSpeed;
	float distance = PointDistance(n1.point, n2);
	//UE_LOG(LogTemp, Display, TEXT("Node with orientation: %f, %f, has to turn %f rads to face target, time to reach is %f"), n1.orientation.X, n1.orientation.Y, angleDistance, turningTime + (distance / Velocity));

	return turningTime + (distance / Velocity);
}

float NodeSelector::GetCosAngle(const FVector& v1, const FVector& v2) {
	FVector dir = v1;
	dir.Z = 0;
	//dir.Normalize();
	FVector point = v2;
	point.Z = 0;
	//point.Normalize();
	//float angledir = atan2(dir.Y, dir.X);
	//float angleToPoint = atan2(point.Y, point.X);
	return (FVector::DotProduct(dir, point)) / (dir.Size() * point.Size());
}

//Always turns and moves forward in maximum speed
Node* NodeSelector::CalculateDifferentialPoint(const Node& n1, const FVector& n2) {
	FVector path = n2 - n1.point;
	float angle = acos(GetCosAngle(n1.orientation, path));
	float orientation = acos(GetCosAngle(FVector(1,0,0), n1.orientation));
	//UE_LOG(LogTemp, Display, TEXT("orientation: %f"), angle);
	if (n1.orientation.Y < 0) 
		orientation = -orientation;
	if (angle > 0.00001) {
		FVector cross = FVector::CrossProduct(path, n1.orientation);
		float partTurn = MaxTurnSpeed * TimeStep / angle;
		if (partTurn > 1)
			partTurn = 1;
		if (cross.Z > 0)
			angle = -angle;
		float turnAngle = angle * partTurn;
		orientation = (turnAngle + orientation);
	}
	//UE_LOG(LogTemp, Display, TEXT("orientation: %f"), orientation);
	FVector newOrientation = FVector(cos(orientation), sin(orientation), n1.point.Z);
	float newX = Velocity * cos(orientation)*TimeStep;
	float newY = Velocity * sin(orientation)*TimeStep;
	FVector newPosition = FVector(n1.point.X + newX, n1.point.Y + newY, n1.point.Z);
	
	//UE_LOG(LogTemp, Display, TEXT("DistanceFromCurrenet: %f, DistanceFromParent: %f"), DifferentialDriveDistance(Node(n1, newPosition, newOrientation), n2), DifferentialDriveDistance(n1, n2));
	if (DifferentialDriveDistance(Node(n1, newPosition,newOrientation), n2) > DifferentialDriveDistance(n1, n2)) {
		newPosition = n1.point;
		newY = 0.0;
		newX = 0.0;

	}
	//UE_LOG(LogTemp, Display, TEXT("PrevLocation: %f, %f, New Location: %f, %f"), n1.point.X, n1.point.Y, newPosition.X, newPosition.Y);
	//UE_LOG(LogTemp, Display, TEXT("PrevOrientation: %f, %f, NewOrientation: %f, %f"), n1.orientation.X, n1.orientation.Y, newOrientation.X, newOrientation.Y);

	return new Node(n1, newPosition, newOrientation);
	



	/*
	float Vx = cos(n1.orientation); //Directional vector of n1
	float Vy = sin(n1.orientation);
	float angleDistance = GetAngle(n1.point,FVector(Vx, Vy, 0.f), n2);
	float cosa = GetCosAngle(n1.point, FVector(Vx, Vy, 0.f), n2);
	
	float turningTime = abs(angleDistance) / MaxTurnSpeed;
	UE_LOG(LogTemp, Display, TEXT("angleDistance %f, curr orientation: %f, nextOrientation: %f or %f"),angleDistance, n1.orientation, n1.orientation + angleDistance, n1.orientation + copysign(1.0, angleDistance)*(MaxTurnSpeed * TimeStep));

	if (TimeStep - turningTime < 0.0) {
		float orientation = n1.orientation + copysign(1.0, angleDistance)*(MaxTurnSpeed * TimeStep);
		//UE_LOG(LogTemp, Display, TEXT("TURN IN PLACE"));
		return new Node(n1, n1.point, orientation);
	}
	float TimeLeft = TimeStep - turningTime;
	float x = n1.point.X + Velocity * cos(n1.orientation + angleDistance)*(TimeLeft);
	float y = n1.point.Y + Velocity * sin(n1.orientation + angleDistance)*(TimeLeft);
	Node * node = new Node(n1, FVector(x, y, n1.point.Z), n1.orientation + angleDistance);
	return node;
	*/

}

// Should work for every model except the car
bool NodeSelector::CheckTrivialPath(const FVector& from, const FVector &to) {
	TArray<FVector> trivialPath = TArray<FVector>();
	if (PointDistance(from, to) <= Velocity * TimeStep) {
		return true;
	}
	float theta = atan2(to.Y - from.Y, to.X - from.X); // WRONG
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
			if (count % 20 == 0) {
				rand.X = EndPosition.X;
				rand.Y = EndPosition.Y;

				//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
			}
			for (int i = 0; i < nodes.Num(); ++i) {
				if (PointDistance(nodes[i]->point, rand) <= PointDistance(parent->point, rand)) {
					FVector NewPoint = CalculatePoint(nodes[i]->point, rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		FVector NewNode = CalculatePoint(parent->point, rand);
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);

		nodes.Add(new Node(parent, NewNode));

		if (PointDistance(NewNode, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), nodes.Num());
}

void NodeSelector::differentialRrt(const FVector EndPosition, const FVector StartPosition, FVector startOrientation, FVector EndOrientation) {
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
				if (returned <= returned2) {
					NewNode = CalculateDifferentialPoint(*nodes[i], rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		//NewNode->point.Z = 70.f;
		nodes.Add(new Node(parent, NewNode->point, NewNode->orientation));
		//UE_LOG(LogTemp, Display, TEXT("New Position: %f, %f"), NewNode->point.X, NewNode->point.Y);
		//UE_LOG(LogTemp, Display, TEXT("New orientat: %f, %f"), NewNode->orientation.X, NewNode->orientation.Y);
		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition, NewNode->orientation));
			//Rotate to the correct position
			float MaxTurnDistance = TimeStep * MaxTurnSpeed;
			//float Vx = cos(EndOrientation);
			//float Vy = sin(EndOrientation);
			int timeswerotate = 0;
			//UE_LOG(LogTemp, Display, TEXT("MaxAngleTurn: %f"), MaxTurnDistance);
			//UE_LOG(LogTemp, Display, TEXT("Endorientation: %f NewNode orientation: %f"), EndOrientation, NewNode->orientation);
			while (abs(EndOrientation.X - nodes[nodes.Num()-1]->orientation.X) < 0.1f || (abs(EndOrientation.Y - nodes[nodes.Num() - 1]->orientation.Y) < 0.1f)){
				//Rotate into place
				timeswerotate++;
				FVector goal = nodes[nodes.Num() - 1]->orientation;
				float dot = FVector::DotProduct(EndOrientation, goal);
				float det = EndOrientation.X * goal.Y - goal .X* EndOrientation.Y;
				float angleDistance = atan2(det, dot);
				//float angleDistance = EndOrientation - nodes[nodes.Num() - 1]->orientation;
				UE_LOG(LogTemp, Display, TEXT("AngleDistance: %f"), angleDistance);
				if (abs(angleDistance) <= MaxTurnDistance) {
					nodes.Add(new Node(nodes[nodes.Num()-1], EndPosition, EndOrientation));
					return;
				}
				nodes.Add(new Node(nodes[nodes.Num()-1], NewNode->point, nodes[nodes.Num() - 1]->orientation + copysign(1.0,-angleDistance)*MaxTurnDistance));
			}
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL, rotated %d times"),timeswerotate);
			count = NumNodes;
		}
		count++;
		//UE_LOG(LogTemp, Display, TEXT("%d"), count);
	}
	UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), nodes.Num());

}

void NodeSelector::GetPath(TArray<Node*>& vectors)
{
	float x;
	float y;
	for (int i = 0; i < PathSize; ++i) {
		x = FMath::RandRange(-XBound, XBound);
		y = FMath::RandRange(-YBound, YBound);
		vectors.Add(new Node(FVector(x, y, 0.f)));
		//UE_LOG(LogTemp, Display, TEXT("Goal X: %f Goal Y: %f"), x, y);
	}
}
