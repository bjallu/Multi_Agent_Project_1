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
	DynamicNodes = TArray<DynamicNode*>();
	obstacles = TArray<Obstacle>();
	StepSize = 20;
	TimeStep = 0.1;
	VehicleLength = 2;
	Velocity = 1.1;
	MaxTurnSpeed = 1.0;
	Acceleration = 1.3;
	GoalVelocity = FVector(0.5f, -0.5f, 0.0f);
}

NodeSelector::~NodeSelector()
{
}
//PATH GETTERS
//run rrt first to create the nodes
void NodeSelector::GetRrtPath(TArray<Node*>& vectors) {
	vectors.Empty();
	//nodes[0] is startposition so shouldnt include it
	Node* CurrentNode = nodes[nodes.Num() - 1];
	while (CurrentNode->point != nodes[0]->point) {
		CurrentNode->point.Z = 0;
		vectors.Add(CurrentNode);
		CurrentNode = CurrentNode->parent;
	}
	UE_LOG(LogTemp, Display, TEXT("TIME: %f"), vectors.Num()*TimeStep);
	Algo::Reverse(vectors);
}

//run rrt first to create the nodes
void NodeSelector::GetDynamicRrtPath(TArray<DynamicNode*>& vectors) {
	vectors.Empty();
	//nodes[0] is startposition so shouldnt include it
	DynamicNode* CurrentNode = DynamicNodes[DynamicNodes.Num() - 1];
	while (CurrentNode->point != DynamicNodes[0]->point) {
		CurrentNode->point.Z = 0;
		vectors.Add(CurrentNode);
		CurrentNode = CurrentNode->parent;
	}
	UE_LOG(LogTemp, Display, TEXT("TIME: %f"), vectors.Num()*TimeStep);
	Algo::Reverse(vectors);
}
//-------------------------------------------------------//
// Kinematic RRT//
//-------------------------------------------------------//
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


//-------------------------------------------------------//
//Differential RRT
//-------------------------------------------------------//
float NodeSelector::DifferentialDriveDistance(const Node& n1, const FVector&n2) {
	//Calculate angle to turn
	FVector path = n2 - n1.point;
	float angleDistance = acos(GetCosAngle(n1.orientation, path));
	float turningTime = angleDistance / MaxTurnSpeed;
	float distance = PointDistance(n1.point, n2);
	return turningTime + (distance / Velocity);
}

float NodeSelector::GetCosAngle(const FVector& v1, const FVector& v2) {
	FVector dir = v1;
	dir.Z = 0;
	FVector point = v2;
	point.Z = 0;
	return (FVector::DotProduct(dir, point)) / (dir.Size() * point.Size());
}

//Always turns and moves forward in maximum speed
Node* NodeSelector::CalculateDifferentialPoint(const Node& n1, const FVector& n2) {
	FVector path = n2 - n1.point;
	float angle = acos(GetCosAngle(n1.orientation, path));
	float orientation = acos(GetCosAngle(FVector(1,0,0), n1.orientation));
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
	FVector newOrientation = FVector(cos(orientation), sin(orientation), n1.point.Z);
	float newX = Velocity * cos(orientation)*TimeStep;
	float newY = Velocity * sin(orientation)*TimeStep;
	FVector newPosition = FVector(n1.point.X + newX, n1.point.Y + newY, n1.point.Z);
	
	if (DifferentialDriveDistance(Node(n1, newPosition,newOrientation), n2) > DifferentialDriveDistance(n1, n2)) {
		newPosition = n1.point;
		newY = 0.0;
		newX = 0.0;

	}
	return new Node(n1, newPosition, newOrientation);
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
					// IF NOT COLLIDES:
					NewNode = CalculateDifferentialPoint(*nodes[i], rand);
					parent = nodes[i];
					foundNext = true;
				}
			}
		}
		nodes.Add(new Node(parent, NewNode->point, NewNode->orientation));
		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			nodes.Add(new Node(parent, EndPosition, NewNode->orientation));
			//Rotate to the correct position
			float MaxTurnDistance = TimeStep * MaxTurnSpeed;
			float timeswerotate = 0.0;
			float angle = acos(GetCosAngle(nodes[nodes.Num() - 1]->orientation, EndOrientation));
			while (!(angle<0.1f)) {
				//Rotate into place
				timeswerotate++;
				//float angleDistance = EndOrientation - nodes[nodes.Num() - 1]->orientation;
				angle = acos(GetCosAngle(nodes[nodes.Num() - 1]->orientation, EndOrientation));
				float angleDistance = acos(GetCosAngle(FVector(1, 0, 0), nodes[nodes.Num() - 1]->orientation));
				UE_LOG(LogTemp, Display, TEXT("orientation: %f"), angle);
				if (nodes[nodes.Num() - 1]->orientation.Y < 0)
					angleDistance = -angleDistance;
				if (angle > 0.00001) {
					FVector cross = FVector::CrossProduct(EndOrientation, nodes[nodes.Num() - 1]->orientation);
					float partTurn = MaxTurnSpeed * TimeStep / angle;
					if (partTurn > 1)
						partTurn = 1;
					if (cross.Z > 0)
						angle = -angle;
					float turnAngle = angle * partTurn;
					angleDistance = (turnAngle + angleDistance);
				}
				if (abs(angleDistance) <= MaxTurnDistance) {
					nodes.Add(new Node(nodes[nodes.Num() - 1], EndPosition, EndOrientation));
					UE_LOG(LogTemp, Display, TEXT("Turned %f times in place"), timeswerotate);
					return;
				}
				FVector newOrientation = FVector(cos(angleDistance), sin(angleDistance), nodes[nodes.Num() - 1]->point.Z);
				nodes.Add(new Node(nodes[nodes.Num() - 1], EndPosition, newOrientation));
				//update angle
				angle = acos(GetCosAngle(nodes[nodes.Num() - 1]->orientation, EndOrientation));
				//
			}
			count = NumNodes;
		}
		count++;
		//UE_LOG(LogTemp, Display, TEXT("%d"), count);
	}
}


//-------------------------------------------------------//
//Dynamic Point RRT
//-------------------------------------------------------//

DynamicNode* NodeSelector::CalculateDynamicPointNode(const DynamicNode& n1, FVector n2) {
	FVector currentVelocity = n1.Velocity;
	FVector direction = n2 - n1.point;
	//Make acceleration towards direction
	direction /= direction.Size(); //Normalize;
	direction *= Acceleration;     //Set to max acceleration
	if (currentVelocity.Size() > Velocity) {
		currentVelocity /= currentVelocity.Size(); //Normalize
		currentVelocity *= Velocity;			//Set to max velocity
		UE_LOG(LogTemp, Display, TEXT("New Velocity: %f"), currentVelocity.Size());
	}
	FVector NewVelocity = currentVelocity + direction;
	NewVelocity = NewVelocity * Velocity / NewVelocity.Size(); //Set length to velocity
	FVector NewPosition = n1.point + NewVelocity * TimeStep;	

	return new DynamicNode(n1,NewPosition,NewVelocity);
}

void NodeSelector::dynamicPointRrt(FVector EndPosition, FVector StartPosition, FVector StartVelocity, FVector EndVelocity) {
	DynamicNodes.Empty();
	//Create startnode
	DynamicNode* StartNode = new DynamicNode(StartPosition, StartVelocity);
	DynamicNodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	DynamicNode* parent = DynamicNodes[0];
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
			for (int i = 0; i < DynamicNodes.Num(); ++i) {
				if (PointDistance(DynamicNodes[i]->point, rand) <= PointDistance(parent->point, rand)) {
					FVector NewPoint = CalculatePoint(DynamicNodes[i]->point, rand);
					parent = DynamicNodes[i];
					foundNext = true;
				}
			}
		}
		FVector NewNode = CalculatePoint(parent->point, rand);
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);

		DynamicNodes.Add(new DynamicNode(parent, NewNode));

		if (PointDistance(NewNode, EndPosition)<GoalRadius) {
			DynamicNodes.Add(new DynamicNode(parent, EndPosition));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), DynamicNodes.Num());
}


