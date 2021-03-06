// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"
#define _CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>  
#include "MapFunctions.h"
#include <cmath>
#include <algorithm>

NodeSelector::NodeSelector()
{
	//maxX = map.bounding_box.maxX;
//	minX = map.bounding_box.minX;
//	maxY = map.bounding_box.maxY;
//	minY = map.bounding_box.minY;
	XBound = 50.f;
	YBound = 50.f;
	PathSize = 1;
	NumNodes = 20000;
	//GoalRadius = 1.0f;
	GoalRadius = 0.3f;
	nodes = TArray<Node*>();
	CarNodes = TArray<CarNode*>();
	DynamicNodes = TArray<DynamicNode*>();
	obstacles = TArray<Obstacle>();
	StepSize = 20;
	TimeStep = 10;
	VehicleLength = 2.0;
	Velocity = 1.1;
	MaxTurnSpeed = 1.0;
	Acceleration = 1.3;
	MaxTurnAngle = 0.6;
	GoalVelocity = FVector(0.5f, -0.5f, 0.0f);
	NrOfCarNodes = 10;
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
		//CurrentNode->point.Z = 0;
		vectors.Add(CurrentNode);
		CurrentNode = CurrentNode->parent;
	}
	UE_LOG(LogTemp, Display, TEXT("TIME: %f"), vectors.Num()*TimeStep);
	Algo::Reverse(vectors);
}

void NodeSelector::getCarRrtPath(TArray<CarNode*>& vectors) {
	vectors.Empty();
	//nodes[0] is startposition so shouldnt include it
	CarNode* CurrentNode = CarNodes[CarNodes.Num() - 1];
	while (CurrentNode->point != CarNodes[0]->point) {
		//CurrentNode->point.Z = 0;
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
		//UE_LOG(LogTemp, Display, TEXT("CurrentNode x, y : %f, %f"), CurrentNode->point.X, CurrentNode->point.Y);
		//CurrentNode->point.Z = 0;
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
	FVector po1 = p1;
	FVector po2 = p2;
	po1.Z = 0.f;
	po2.Z = po1.Z;
	FVector dir = po2 - po1;
	//dir.Z = 0;
	//dir = (dir / dir.Size()) * Velocity * TimeStep; //Normalize
	dir = (dir / dir.Size()) * 1;
	// maybe add a constant velocity 
	float theta = atan2(p2.Y - p1.Y, p2.X - p1.X);
	return p1 + dir;
}

void NodeSelector::rrt(const FVector EndPosition, const FVector StartPosition, FVector startOrientation, FVector EndOrientation, MapFunctions map) {
	nodes.Empty();
	//Create startnode
	TimeStep = map.vehicle_dt;
	VehicleLength = map.vehicle_L;
	Velocity = map.vehicle_v_max;
	MaxTurnSpeed = map.vehicle_omega_max;
	Acceleration = map.vehicle_a_max;
	MaxTurnAngle = map.vehicle_phi_max;
	Node* StartNode = new Node(StartPosition);
	nodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	float minX = map.bounding_box.minX;
	float maxX = map.bounding_box.maxX;
	float minY = map.bounding_box.minY;
	float maxY = map.bounding_box.maxY;
	//UE_LOG(LogTemp, Display, TEXT("end: %f, %f"), EndPosition.X, EndPosition.Y);
	//UE_LOG(LogTemp, Display, TEXT("start: %f, %f"), StartPosition.X, StartPosition.Y);
	FVector NewPoint;
	FVector coordinates;
	FVector rand = FVector(x, y, map.z);
	Node* parent = nodes[0];
	while (count < NumNodes) {
		//Check if there is a straight line to the target from the current position
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(minX, maxX);
			rand.Y = FMath::RandRange(minY, maxY);
			// First check if its in bounding box if it is continue
			if (map.OutsideBoundingBoxCheck(rand)) continue;
			if (count % 3000 == 0) { // try 20%
				rand.X = EndPosition.X;
				rand.Y = EndPosition.Y;

				//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
			}
			//UE_LOG(LogTemp, Display, TEXT("STILL SAMPLING"));
			for (int i = 0; i < nodes.Num(); ++i) {
				if (PointDistance(nodes[i]->point, rand) <= PointDistance(parent->point, rand)) {
					coordinates = CalculatePoint(nodes[i]->point, rand);
					//UE_LOG(LogTemp, Display, TEXT("Coordinates: %f, %f, %s"),coordinates.X, coordinates.Y, map.ObstacleCollisionCheck(coordinates)? TEXT("True"): TEXT("False"));

					if (!map.ObstacleCollisionCheck(coordinates)) {

						//UE_LOG(LogTemp, Display, TEXT("do we ever get here"));
						NewPoint = coordinates;
						parent = nodes[i];
						foundNext = true;
					}
				}
			}
		}
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);
		

		nodes.Add(new Node(parent, NewPoint));
		//UE_LOG(LogTemp, Display, TEXT("NewPoint: %f, %f"), NewPoint.X, NewPoint.Y);
		//UE_LOG(LogTemp, Display, TEXT("distance to goal: %f"), PointDistance(NewPoint, EndPosition));
		if (PointDistance(NewPoint, EndPosition)<Velocity*TimeStep) {
			nodes.Add(new Node(parent, EndPosition));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			return;
			count = NumNodes;
		}
		//Draw debug line
		UE_LOG(LogTemp, Display, TEXT("nodes: %d"), count);
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

void NodeSelector::differentialRrt(const FVector EndPosition, const FVector StartPosition, FVector startOrientation, FVector EndOrientation, MapFunctions map) {
	nodes.Empty();
	//Create startnode
	TimeStep = map.vehicle_dt;
	VehicleLength = map.vehicle_L;
	Velocity = map.vehicle_v_max;
	MaxTurnSpeed = map.vehicle_omega_max;
	Acceleration = map.vehicle_a_max;
	MaxTurnAngle = map.vehicle_phi_max;
	//map.vehicle_t;
	//Load all values from the map
	Node* StartNode = new Node(StartPosition, startOrientation);
	nodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = EndPosition.X;
	float y = EndPosition.Y;
	FVector rand = FVector(x, y, StartPosition.Z);
	Node* parent = nodes[0];
	Node* NewNode = parent;

	float minX = map.bounding_box.minX;
	float maxX = map.bounding_box.maxX;
	float minY = map.bounding_box.minY;
	float maxY = map.bounding_box.maxY;

	while (count < NumNodes) {
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(minX, maxX);
			rand.Y = FMath::RandRange(minY, maxY);
			// First check if its in bounding box if it is continue
			if (map.OutsideBoundingBoxCheck(rand)) continue;
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
					Node *nodeToTest;
					nodeToTest = CalculateDifferentialPoint(*nodes[i], rand);
					FVector coordinates = nodeToTest->point;
					if (!map.ObstacleCollisionCheck(coordinates)) {

						//UE_LOG(LogTemp, Display, TEXT("do we ever get here"));
						NewNode = CalculateDifferentialPoint(*nodes[i], rand);
						parent = nodes[i];
						foundNext = true;
					}
					
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
	direction *= Acceleration * TimeStep;     //Set to max acceleration
	FVector NewVelocity = currentVelocity + direction;
	//NewVelocity = NewVelocity * Velocity / NewVelocity.Size(); //Set length to velocity
	if (NewVelocity.Size() > Velocity) {
		NewVelocity /= currentVelocity.Size(); //Normalize
		NewVelocity *= Velocity;			//Set to max velocity
	}
	FVector NewPosition = n1.point + NewVelocity * TimeStep;
	return new DynamicNode(n1,NewPosition,NewVelocity);
}


TArray<DynamicNode*> NodeSelector::DynamicDubinsMove(const DynamicNode& n1, const DynamicNode& n2, const float R1, const float R2, const FVector tp1, const FVector tp2, const FVector p1, const FVector p2, bool rightStart, bool rightGoal, MapFunctions map, const UWorld* world) {
	TArray<DynamicNode*> next;


	FVector tangentline = tp2 - tp1;

	if (map.ObstacleCollisionCheck(tp2) || map.ObstacleCollisionCheck(tp1)) return next;
	float arcLength = getArcLength(n1.point - p1, tp1 - p1, !rightStart, R1);


	float movingDTs = arcLength / (TimeStep * n1.Velocity.Size());
	float moveCount = ceil(movingDTs);	//How many times to move using the current velocity
	float phi1 = arcLength / R1;		//the distance between two points of in the circle with radius R1
	float angleStep = phi1 / moveCount; //The difference in the angle between two consecutive points from the circle center
	float rotateRight = rightStart ? 1 : -1;
	float baseAngle = acos(GetCosAngle(FVector(1,0,0),n1.point - p1));

	//float baseAngle = atan2((n1.point - p1).X, (n1.point - p1).Y);
	if ((n1.point - p1).Y < 0) baseAngle = -baseAngle;




	float newX = p1.X + R1 * cos(baseAngle + rotateRight * angleStep);
	float newY = p1.Y + R1 * sin(baseAngle + rotateRight * angleStep);
	FVector newPosition = FVector(newX, newY, 0.0);
	FVector newVelocity = FVector((newX - n1.point.X) / TimeStep, (newY - n1.point.Y) / TimeStep, 0.0f); // new velocity vector should be the distance we travelled between the last point to current split by the time we travelled
	next.Add(new DynamicNode(n1, newPosition, newVelocity));
	if (map.ObstacleCollisionCheck(newPosition)) {
		next.Empty();
		return next;
	}
	// [0] is the current position
	for (int i = 2; i <= moveCount; ++i) {
		float newX = p1.X + R1 * cos(baseAngle + rotateRight * i * angleStep);
		float newY = p1.Y + R1 * sin(baseAngle + rotateRight * i * angleStep);
		FVector newPosition = FVector(newX, newY, map.z);
		FVector newVelocity = FVector((newX - next[next.Num() - 1]->point.X) / TimeStep, (newY - next[next.Num() - 1]->point.Y)/TimeStep, 0.0f); // new velocity vector should be the distance we travelled between the last point to current split by the time we travelled
		next.Add(new DynamicNode(next[next.Num() - 1],newPosition, newVelocity));

		if (map.ObstacleCollisionCheck(newPosition)) {
			next.Empty();
			return next;
		}
	}
	FVector sphereVector = next[next.Num() - 1]->point;
	sphereVector.Z = map.z;
	next.Add(new DynamicNode(next[next.Num() - 1],tp1,(tangentline/tangentline.Size())*n1.Velocity.Size())); //add tangent point with the velocity in direction of the tangent line with size of n1
	

	//Move forward
	float movingDtsDacc = (n2.Velocity - next[next.Num()-1]->Velocity).Size() / Acceleration; //the time we need to deaccelerate or accelerate to goal velocity
	float moveCountDacc = ceil(movingDtsDacc);

	float coefficient = n2.Velocity.Size() > next[next.Num() - 1]->Velocity.Size() ? 1 : -1; //Deaccelerate or accelerate
	float distance = 0.0f;
	float vel = next[next.Num() - 1]->Velocity.Size();
	
	for (int i = 0; i < moveCountDacc; ++i) {
		distance += vel * TimeStep;						   //Distance we travel through the tangent line until we start acceleration/deacceleration
		vel = vel + coefficient * Acceleration * TimeStep; //Assuming that the goal velocity is not higher than our allowed velocity
	}
	if (distance > tangentline.Size()) {
		next.Empty();
		return next;
	}
	
	movingDTs = (tangentline.Size() - distance) / (next[next.Num() - 1]->Velocity.Size() * TimeStep); //Time we use to deacellerate or accelerate
	moveCount = ceil(movingDTs);

	
	//Move on tangent line
	// PROBLEM BETWEEN HERE
	
	for (int i = 1; i < moveCount; ++i) {
		FVector NewPosition = tp1 + i * TimeStep * next[next.Num() - 1]->Velocity.Size() * (tangentline / tangentline.Size());
		next.Add(new DynamicNode(next[next.Num()-1],NewPosition,(tangentline/tangentline.Size())* next[next.Num()-1]->Velocity.Size()));
		if (map.ObstacleCollisionCheck(NewPosition)) {
			next.Empty();
			return next;
		}
	}
	next.Add(new DynamicNode(next[next.Num() - 1], tp2 - (tangentline / tangentline.Size()) * distance, (tangentline/tangentline.Size()) * next[next.Num()-1]->Velocity.Size()));
	if (map.ObstacleCollisionCheck(tp2 - (tangentline / tangentline.Size()) * distance)) {
		next.Empty();
		return next;
	}
	
	// Now we add the accelerate / deaccelerate nodes
	
	for (int i = 1; i < moveCountDacc; ++i) {
		vel = (next[next.Num() - 1]->Velocity.Size() + coefficient * Acceleration) * TimeStep;
		FVector NewPosition = next[next.Num() - 1]->point + (tangentline / tangentline.Size()) * vel;
		next.Add(new DynamicNode(next[next.Num() - 1], NewPosition, (tangentline / tangentline.Size())*vel/TimeStep));
		if (map.ObstacleCollisionCheck(NewPosition)) {
			next.Empty();
			return next;
		}
	}
	// AND HERE
	//Add tangent point 2
	next.Add(new DynamicNode(next[next.Num()-1], tp2, n2.Velocity.Size()* (tangentline / tangentline.Size())));


	arcLength = getArcLength(tp2 - p2, n2.point - p2, !rightGoal, R2);
	movingDTs = arcLength / (TimeStep * n2.Velocity.Size());
	moveCount = int(ceil(movingDTs));
	phi1 = arcLength / R2;
	angleStep = phi1 / moveCount;
	baseAngle = acos(GetCosAngle(FVector(1,0,0),tp2-p2));
	if ((tp2 - p2).Y < 0) baseAngle = -baseAngle;
	rotateRight = rightGoal ? 1 : -1;
	for (int i = 1; i < moveCount; ++i) {
		float newX = p2.X + R2 * cos(baseAngle + rotateRight * i * angleStep);
		float newY = p2.Y + R2 * sin(baseAngle + rotateRight * i * angleStep);
		FVector NewPosition = FVector(newX, newY, 0.0);
		FVector NewVelocity = FVector((newX - next[next.Num()-1]->point.X )/ TimeStep, (newY - next[next.Num() - 1]->point.Y) / TimeStep, 0.0f);
		next.Add(new DynamicNode(next[next.Num() - 1], NewPosition, NewVelocity));
		if (map.ObstacleCollisionCheck(NewPosition)) {
			next.Empty();
			return next;
		}
	}
	next.Add(new DynamicNode(next[next.Num()-1],n2.point,n2.Velocity));
	

	return next;

}

TArray<DynamicNode*> NodeSelector::TangentsDynamicPoint(DynamicNode& n1,DynamicNode& n2, MapFunctions map, const UWorld* world) {

	// Left left
	TArray<DynamicNode*> shortest;
	TArray<DynamicNode*> next;
	//float R = VehicleLength / tan(MaxTurnAngle);

	float n1Velocity = sqrt(n1.Velocity.X*n1.Velocity.X + n1.Velocity.Y*n1.Velocity.Y);
	float n2Velocity = sqrt(n2.Velocity.X*n2.Velocity.X + n2.Velocity.Y*n2.Velocity.Y);

	float R1 = pow(n1Velocity, 2) / map.vehicle_a_max;
	float R2 = pow(n2Velocity, 2) / map.vehicle_a_max;

	FVector cross = FVector::CrossProduct(n1.Velocity, FVector(0, 0, 1));
	cross = (cross / cross.Size());
	n2.Velocity.Z = n1.Velocity.Z;
	FVector cross2 = FVector::CrossProduct(n2.Velocity, FVector(0, 0, 1));
	cross2 = (cross2 / cross2.Size());

	FVector A = n1.point + (cross)*R1;
	FVector D = n2.point + (cross2)*R2;
	A.Z = D.Z;

	float distance = (D - A).Size();
	float H = sqrt(pow(distance, 2) - pow((R1 - R2), 2));
	float Y = sqrt(pow(H, 2) + pow(R2, 2));
	float theta = acos((pow(R1, 2) + pow(distance, 2) - pow(Y, 2)) / (2 * R1*distance));
	//float angleatan2 = atan2(D.Y - A.Y, D.X - A.X); // might need to use Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
	float angleatan2 = acos(GetCosAngle(FVector(1, 0, 0), D - A));											//	if ((p2 - p1).z < 0)
	if ((D - A).Y < 0) angleatan2 = -angleatan2;					//baseAngle = -baseAngle;
													//
	FVector B = FVector(A.X + R1 * cos(theta + angleatan2), A.Y + R1 * sin(theta + angleatan2), 0.0f);
	FVector C = FVector(D.X + R2 * cos(theta + angleatan2), D.Y + R2 * sin(theta + angleatan2), 0.0f);
	next = DynamicDubinsMove(n1, n2, R1, R2, B, C, A, D, false, false, map, world);
	if (next.Num() != 0) shortest = next;
	
	//right right pretty tight
	cross = FVector::CrossProduct(n1.Velocity, FVector(0, 0, 1));
	cross = (cross / cross.Size());
	n2.Velocity.Z = n1.Velocity.Z;
	cross2 = FVector::CrossProduct(n2.Velocity, FVector(0, 0, 1));
	cross2 = (cross2 / cross2.Size());

	A = n1.point - (cross)*R1;
	D = n2.point - (cross2)*R2;
	A.Z = D.Z;

	distance = (D - A).Size();
	H = sqrt(pow(distance, 2) - pow((R1 - R2), 2));
	Y = sqrt(pow(H, 2) + pow(R2, 2));
	theta = -acos((pow(R1, 2) + pow(distance, 2) - pow(Y, 2)) / (2 * R1*distance));
	angleatan2 = acos(GetCosAngle(FVector(1, 0, 0), D - A)); // might need to use Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
	if ((D - A).Y < 0) angleatan2 = -angleatan2;					//baseAngle = -baseAngle;
	B = FVector(A.X + cos(theta + angleatan2)*R1, A.Y + sin(theta + angleatan2)*R1, 0.0f);
	C = FVector(D.X + cos(theta + angleatan2)*R2, D.Y + sin(theta + angleatan2)*R2, 0.0f);
	next = DynamicDubinsMove(n1, n2, R1, R2, B, C, A, D, true, true, map, world);
	if (next.Num() != 0 && next.Num()<shortest.Num()) shortest = next;


	A = n1.point + (cross)*R1;
	D = n2.point - (cross2)*R2;
	A.Z = D.Z;

	distance = (D - A).Size();
	H = sqrt(pow(distance, 2) - pow((R1 - R2), 2));
	Y = sqrt(pow(H, 2) + pow(R2, 2));
	theta = acos((pow(R1, 2) + pow(distance, 2) - pow(Y, 2)) / (2 * R1*distance));
	angleatan2 = acos(GetCosAngle(FVector(1, 0, 0), D - A)); // might need to use Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
	if ((D - A).Y < 0) angleatan2 = -angleatan2;					//baseAngle = -baseAngle;
	B = FVector(A.X + cos(theta + angleatan2)*R1, A.Y + sin(theta + angleatan2)*R1, 0.0f);
	C = FVector(D.X - cos(theta + angleatan2)*R2, D.Y - sin(theta + angleatan2)*R2, 0.0f);
	next = DynamicDubinsMove(n1, n2, R1, R2, B, C, A, D, false, true, map, world);
	if (next.Num() != 0 && next.Num()<shortest.Num()) shortest = next;


	A = n1.point - (cross)*R1;
	D = n2.point + (cross2)*R2;
	A.Z = D.Z;

	distance = (D - A).Size();
	H = sqrt(pow(distance, 2) - pow((R1 - R2), 2));
	Y = sqrt(pow(H, 2) + pow(R2, 2));
	theta = -acos((pow(R1, 2) + pow(distance, 2) - pow(Y, 2)) / (2 * R1*distance));
	angleatan2 = acos(GetCosAngle(FVector(1, 0, 0), D - A)); // might need to use Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
	if ((D - A).Y < 0) angleatan2 = -angleatan2;					//baseAngle = -baseAngle;
	B = FVector(A.X + cos(theta + angleatan2)*R1, A.Y + sin(theta + angleatan2)*R1, 0.0f);
	C = FVector(D.X - cos(theta + angleatan2)*R2, D.Y - sin(theta + angleatan2)*R2, 0.0f);
	next = DynamicDubinsMove(n1, n2, R1, R2, B, C, A, D, true, false, map, world);
	if (next.Num() != 0 && next.Num()<shortest.Num()) shortest = next;


	return shortest;
}

float NodeSelector::getArcLength(FVector v1, FVector v2, bool right, float radius){
	float theta = atan2(v1.X, v1.Y) - atan2(v2.X, v2.Y);
	if (theta < 0.f && !right)
		theta = theta + 2 * PI;
	else if (theta > 0.f && right)
		theta = theta - 2 * PI;
	return abs(theta * radius);
}

float NodeSelector::GetDynamicPointDistance(const DynamicNode& n1, const FVector n2) {
	float distance = PointDistance(n1.point, n2);
	FVector norm = n1.Velocity / n1.Velocity.Size();
	float angleDistance = atan2(n2.Y - norm.Y, n2.X - norm.X);
	//float angleDistance = atan2(n2.Y, n2.X) - atan2(n1.orientation.Y, n1.orientation.X);
	//float angleDistance = acos(GetCosAngle(n1.orientation,n2));
	float R = VehicleLength / tan(MaxTurnAngle);
	float angledistance = R * angleDistance;
	return angleDistance + distance;
}

void NodeSelector::dynamicPointRrt(FVector EndPosition, FVector StartPosition, FVector StartVelocity, FVector EndVelocity, MapFunctions map, const UWorld* world) {
	DynamicNodes.Empty();
	TimeStep = map.vehicle_dt;
	VehicleLength = map.vehicle_L;
	Velocity = map.vehicle_v_max;
	MaxTurnSpeed = map.vehicle_omega_max;
	Acceleration = map.vehicle_a_max;
	MaxTurnAngle = map.vehicle_phi_max;
	float minX = map.bounding_box.minX;
	float maxX = map.bounding_box.maxX;
	float minY = map.bounding_box.minY;
	float maxY = map.bounding_box.maxY;
	//Create startnode
	DynamicNode* StartNode = new DynamicNode(StartPosition, StartVelocity);
	DynamicNode* GoalNode = new DynamicNode(EndPosition, EndVelocity);
	DynamicNodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	DynamicNode* parent = DynamicNodes[0];
	DynamicNode* nodeToCompare;
	DynamicNode* NewNode = parent;

	
	
	while (count < NumNodes) {
		//Check if there is a straight line to the target from the current position
		foundNext = false;
		while (!foundNext) {
			rand.X = FMath::RandRange(minX, maxX);
			rand.Y = FMath::RandRange(minY, maxY);
			if (map.OutsideBoundingBoxCheck(rand)) continue;
			if (count % 500 == 0) { //500 seems to be amazing
				rand.X = EndPosition.X;
				rand.Y = EndPosition.Y;

				//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
			}
			for (int i = 0; i < DynamicNodes.Num(); ++i) {
				if (GetDynamicPointDistance(*DynamicNodes[i], rand) <= GetDynamicPointDistance(*parent, rand)) { //ADD COLLISION CHECKT
					nodeToCompare = CalculateDynamicPointNode(*DynamicNodes[i], rand);
					if (!map.ObstacleCollisionCheck(nodeToCompare->point)) {
						NewNode = CalculateDynamicPointNode(*DynamicNodes[i], rand);
						parent = DynamicNodes[i];
						foundNext = true;
					}
				}
			}
		}
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);



		DynamicNodes.Add(new DynamicNode(parent, NewNode->point,NewNode->Velocity));
		TArray<DynamicNode*> trivial = TangentsDynamicPoint(*NewNode, *GoalNode, map, world);
		UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), trivial.Num());

		if (trivial.Num() != 0) {
			trivial[0]->parent = DynamicNodes[DynamicNodes.Num() - 1];
			for (int i = 0; i < trivial.Num(); ++i) {
				DynamicNodes.Add(new DynamicNode(trivial[i]->parent, trivial[i]->point, trivial[i]->Velocity));
			}
			return;
		}
		UE_LOG(LogTemp, Display, TEXT("nodes: %d"), count);
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), DynamicNodes.Num());
	//UE_LOG(LogTemp, Display, TEXT("Node 2 point: %f, %f"), DynamicNodes[2]->point.X, DynamicNodes[2]->point.Y);
}


//-------------------------------------------------------//
//Kinematic Car RRT
//-------------------------------------------------------//


CarNode* NodeSelector::CalculateCarNode(const CarNode&n1, const FVector n2) {
	FVector path = n2 - n1.point;
	float angle = acos(GetCosAngle(n1.orientation, path));
	float orientation = acos(GetCosAngle(FVector(1, 0, 0), n1.orientation));
	if (n1.orientation.Y < 0)
		orientation = -orientation;
	float turn = 0.0f;
	if (angle > 0.00001) {
		FVector cross = FVector::CrossProduct(path, n1.orientation);
		float partTurn = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep / angle;
		if (partTurn < 1.f)
			turn = MaxTurnAngle * copysign(1.0, -cross.Z);
		else
			turn = atan(angle*VehicleLength / Velocity)*copysign(1.0, -cross.Z);
	}
	float delta = (Velocity / VehicleLength)*tan(turn)*TimeStep;
	orientation = delta + orientation;
	FVector newOrientation = FVector(cos(orientation), sin(orientation), n1.point.Z);
	float newX = Velocity * cos(orientation)*TimeStep;
	float newY = Velocity * sin(orientation)*TimeStep;
	FVector newPosition = FVector(n1.point.X + newX, n1.point.Y + newY, n1.point.Z);
	return new CarNode(n1, newPosition, newOrientation);
}



float NodeSelector::ArcLength(FVector center, FVector left, FVector right, float radius, bool isleft) {
	FVector vec1, vec2;
	vec1 = left - center;
	vec2 = right - center;
	float theta = atan2(vec2.Y, vec2.X) - atan2(vec1.Y, vec1.X);
	if (theta < 0.0 && isleft)
		theta += 2.0*PI;
	else if (theta > 0.0 && !isleft)
		theta -= 2.0 * PI;
	return fabs(theta*radius);
}




TArray<CarNode*> NodeSelector::CalculateTangentPoints(CarNode& n1, CarNode& n2, MapFunctions map, const UWorld* world) {

	//New Guide
	TArray<CarNode*> shortest;
	TArray<CarNode*> next;
	float R = VehicleLength / tan(MaxTurnAngle);
	FVector cross = FVector::CrossProduct(n1.orientation, FVector(0, 0, 1));
	cross = (cross / cross.Size())*R;
	n2.orientation.Z = n1.orientation.Z;
	FVector cross2 = FVector::CrossProduct(n2.orientation, FVector(0, 0, 1));
	cross2 = (cross2 / cross2.Size())*R;


	FVector A = n1.point + (cross);
	FVector D = n2.point + (cross2);
	A.Z = D.Z;
	//DrawDebugSphere(world, A, 0.5, 26, FColor::Blue, true);
	//DrawDebugSphere(world, D, 0.5, 26, FColor::Blue, true);
	float theta = PI / 2;
	float angleatan2 = atan2(D.Y - A.Y, D.X - A.X);
	theta += angleatan2;
	FVector B = FVector(A.X + R * cos(theta), A.Y + R * sin(theta), 0.0f);
	FVector C = B + (D - A);
	FVector tangent = D - A;
	next = TraverseDubins(n1, n2, A, D, R, B, C, false, true, -1, -1, map);
	
	
	shortest = next;
	A = n1.point - (cross);
	D = n2.point - (cross2);
	A.Z = D.Z;

	theta = (PI / 2)+ PI;
	angleatan2 = atan2(D.Y - A.Y, D.X - A.X);
	theta += angleatan2;
	B = FVector(A.X + R * cos(theta), A.Y + R * sin(theta), 0.0f);
	C = B + (D - A);
	tangent = D - A;
	next = TraverseDubins(n1, n2, A, D, R, B, C, true, false, 1, 1, map);	
	if (shortest.Num() > next.Num() && next.Num() != 0) {
		shortest = next;
	}
	return shortest;
}


TArray<CarNode*> NodeSelector::LR(CarNode& n1, CarNode& n2, MapFunctions map, const UWorld* world) {
	//Right Straight Left
	float sign = 1;
	//if (leftFirst) sign = 1;
	TArray<CarNode*> next;
	TArray<CarNode*> shortest;
	float R = VehicleLength / tan(MaxTurnAngle);
	FVector cross = FVector::CrossProduct(n1.orientation, FVector(0, 0, 1));
	cross = (cross / cross.Size())*R;
	n2.orientation.Z = n1.orientation.Z;
	FVector cross2 = FVector::CrossProduct(n2.orientation, FVector(0, 0, 1));
	cross2 = (cross2 / cross2.Size())*R;
	FVector A = n1.point + (cross);
	FVector D = n2.point - (cross2);
	float delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
	A.Z = D.Z;
	FVector d = D - A;
	float theta = acos((2*R)/d.Size());
	float angleatan2 = atan2(D.Y - A.Y, D.X - A.X);
	theta += angleatan2;
	FVector B = FVector(A.X + R * cos(theta), A.Y + R * sin(theta), 0.0f);
	FVector C = FVector(A.X + 2*R*cos(theta), A.Y + 2*R*sin(theta), 0.0f);
	FVector direction_to_tangent = D-C;
	float distance2 = sqrt(pow((D.X - C.X), 2.0) + pow((D.Y - C.Y), 2.0));
	FVector E = B + direction_to_tangent;
	FVector tangent = E - B;//E + distance2;
	next = TraverseDubins(n1, n2, A, D, R, B, E, false, false, -1, 1, map);
	return next;
}




TArray<CarNode*> NodeSelector::TraverseDubins(const CarNode& n1, const CarNode& n2, const FVector &A, const FVector &D, const float R, const FVector B, const FVector C, bool first, bool second, float firstDelta, float secondDelta, MapFunctions map) {
	TArray<CarNode*> next;
	FVector tangent = C - B;
	float delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
	float arcl = ArcLength(A, n1.point, B, R, first);
	float timesteps = ceil(arcl / (Velocity * TimeStep));
	float orientation = acos(GetCosAngle(FVector(1, 0, 0), n1.orientation));
	if (n1.orientation.Y < 0)
		orientation = -orientation;
	orientation = (delta*firstDelta) + orientation; //plus delta cus turning right
	FVector newOrientation = FVector(cos(orientation), sin(orientation), n1.point.Z);
	float newX = Velocity * cos(orientation)*TimeStep;
	float newY = Velocity * sin(orientation)*TimeStep;
	FVector newPosition = FVector(n1.point.X + newX, n1.point.Y + newY, n1.point.Z);
	next.Add(new CarNode(n1, newPosition, newOrientation));
	if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
		//for (int i = 0; i < next.Num(); ++i) delete next[i];
		next.Empty();
		return next;
	}
	for (int i = 1; i < timesteps; ++i) {
		orientation = acos(GetCosAngle(FVector(1, 0, 0), next[i - 1]->orientation));
		if (next[i - 1]->orientation.Y < 0)
			orientation = -orientation;
		orientation = (delta*firstDelta) + orientation; //THIS MIGHT BE TURN RIGHT
		newOrientation = FVector(cos(orientation), sin(orientation), next[i - 1]->point.Z);
		newX = Velocity * cos(orientation)*TimeStep;
		newY = Velocity * sin(orientation)*TimeStep;
		newPosition = FVector(next[i - 1]->point.X + newX, next[i - 1]->point.Y + newY, next[i - 1]->point.Z);
		next.Add(new CarNode(next[i - 1], newPosition, newOrientation));
		if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
			//for (int i = 0; i < next.Num(); ++i) delete next[i];
			next.Empty();
			return next;
		}
	}
	next.Add(new CarNode(next[next.Num() - 1], B, tangent / tangent.Size()));
	if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
		//for (int i = 0; i < next.Num(); ++i) delete next[i];
		next.Empty();
		return next;
	}
	orientation = acos(GetCosAngle(FVector(1, 0, 0), tangent / tangent.Size()));
	if (tangent.Y < 0)
		orientation = -orientation;
	float arcL2 = sqrt(pow(tangent.X, 2) + pow(tangent.Y, 2));
	//UE_LOG(LogTemp, Display, TEXT("Arcl2: %f"), arcL2);
	timesteps = ceil(arcL2 / (Velocity * TimeStep));
	newX = Velocity * cos(orientation)*TimeStep;
	newY = Velocity * sin(orientation)*TimeStep;
	newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
	next.Add(new CarNode(next[next.Num() - 1], newPosition, tangent / tangent.Size()));
	if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
		//for (int i = 0; i < next.Num(); ++i) delete next[i];
		next.Empty();
		return next;
	}
	for (int i = 1; i < timesteps; ++i) {
		newX = Velocity * cos(orientation)*TimeStep;
		newY = Velocity * sin(orientation)*TimeStep;
		newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
		next.Add(new CarNode(next[next.Num() - 1], newPosition, tangent / tangent.Size()));
		if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
			//for (int i = 0; i < next.Num(); ++i) delete next[i];
			next.Empty();
			return next;
		}
	}
	next.Add(new CarNode(next[next.Num() - 1], C, tangent / tangent.Size()));
	if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
		//for (int i = 0; i < next.Num(); ++i) delete next[i];
		next.Empty();
		return next;
	}
	// TURN LEFT
	//GO LEFT
	float arcL3 = ArcLength(D, n2.point, C, R, second);
	timesteps = ceil(arcL3 / (Velocity * TimeStep));
	delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
	orientation = acos(GetCosAngle(FVector(1, 0, 0), next[next.Num() - 1]->orientation));
	if (next[next.Num() - 1]->orientation.Y < 0)
		orientation = -orientation;
	orientation = (delta*secondDelta) + orientation;
	newOrientation = FVector(cos(orientation), sin(orientation), next[next.Num() - 1]->point.Z);
	newX = Velocity * cos(orientation)*TimeStep;
	newY = Velocity * sin(orientation)*TimeStep;
	newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
	next.Add(new CarNode(next[next.Num() - 1], newPosition, newOrientation));
	if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
		//for (int i = 0; i < next.Num(); ++i) delete next[i];
		next.Empty();
		return next;
	}
	for (int i = 1; i < timesteps; ++i) {
		orientation = acos(GetCosAngle(FVector(1, 0, 0), next[next.Num() - 1]->orientation));
		if (next[next.Num() - 1]->orientation.Y < 0)
			orientation = -orientation;
		orientation = (delta*secondDelta) + orientation; //THIS MIGHT BE TURN RIGHT
		newOrientation = FVector(cos(orientation), sin(orientation), next[next.Num() - 1]->point.Z);
		newX = Velocity * cos(orientation)*TimeStep;
		newY = Velocity * sin(orientation)*TimeStep;
		newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
		next.Add(new CarNode(next[next.Num() - 1], newPosition, newOrientation));
		if (map.ObstacleCollisionCheck(next[next.Num() - 1]->point) || map.OutsideBoundingBoxCheck(next[next.Num() - 1]->point)) {
			//for (int i = 0; i < next.Num(); ++i) delete next[i];
			next.Empty();
			return next;
		}
		
	}
	next.Add(new CarNode(next[next.Num() - 1], n2.point, n2.orientation));

	return next;
}

TArray<CarNode*> NodeSelector::RL(CarNode& n1, CarNode& n2, MapFunctions map, const UWorld* world) {
	//Right Straight Left
	float sign = 1;
	//if (leftFirst) sign = 1;
	TArray<CarNode*> next;
	TArray<CarNode*> shortest;
	float R = VehicleLength / tan(MaxTurnAngle);
	FVector cross = FVector::CrossProduct(n1.orientation, FVector(0, 0, 1));
	cross = (cross / cross.Size())*R;
	n2.orientation.Z = n1.orientation.Z;
	FVector cross2 = FVector::CrossProduct(n2.orientation, FVector(0, 0, 1));
	cross2 = (cross2 / cross2.Size())*R;
	FVector A = n1.point - (cross);
	FVector D = n2.point + (cross2);
	float delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
	A.Z = D.Z;
	FVector d = D - A;
	float theta = acos((2 * R) / d.Size());
	float angleatan2 = atan2(D.Y - A.Y, D.X - A.X);
	theta = -theta;
	theta += angleatan2;
	//theta = -theta;
	FVector B = FVector(A.X + R * cos(theta), A.Y + R * sin(theta), 0.0f);
	FVector C = FVector(A.X + 2 * R*cos(theta), A.Y + 2 * R*sin(theta), 0.0f);
	FVector direction_to_tangent = D - C;
	float distance2 = sqrt(pow((D.X - C.X), 2.0) + pow((D.Y - C.Y), 2.0));
	FVector E = B + direction_to_tangent;
	FVector tangent = E - B;
	B.Z = map.z;
	next = TraverseDubins(n1, n2, A, D, R, B, E, true, true, 1, -1, map);
	return next;

}

float NodeSelector::GetCarDistance(const CarNode& n1, const FVector n2) {
	float distance = PointDistance(n1.point, n2);
	float angleDistance = atan2(n2.Y - n1.orientation.Y, n2.X - n1.orientation.X);
	//float angleDistance = atan2(n2.Y, n2.X) - atan2(n1.orientation.Y, n1.orientation.X);
	//float angleDistance = acos(GetCosAngle(n1.orientation,n2));
	float R = VehicleLength / tan(MaxTurnAngle);
	float angledistance = R * angleDistance;
	return angleDistance + distance;
}


void NodeSelector::carRrt(FVector EndPosition, FVector StartPosition, FVector StartOrientation, FVector EndOrientation, MapFunctions map, const UWorld * world) {
	CarNodes.Empty();
	TimeStep = map.vehicle_dt;
	VehicleLength = map.vehicle_L;
	Velocity = map.vehicle_v_max;
	MaxTurnSpeed = map.vehicle_omega_max;
	Acceleration = map.vehicle_a_max;
	MaxTurnAngle = map.vehicle_phi_max;
	DrawDebugSphere(world, EndPosition, 0.5, 32, FColor::Yellow, true);
	UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
	//Create startnode
	CarNode* GoalNode = new CarNode(EndPosition, EndOrientation);
	CarNode* StartNode = new CarNode(StartPosition, StartOrientation);
	CarNodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	CarNode* parent = CarNodes[0];
	CarNode* NewNode = parent;

	float minX = map.bounding_box.minX;
	float maxX = map.bounding_box.maxX;
	float minY = map.bounding_box.minY;
	float maxY = map.bounding_box.maxY;

	//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
	while (count < NumNodes) {
		//Check if there is a straight line to the target from the current position
		//UE_LOG(LogTemp, Display, TEXT("Sampling  node"));
		foundNext = false;
		while (!foundNext) {
			//UE_LOG(LogTemp, Display, TEXT("Sampling  node"));
			rand.X = FMath::RandRange(minX, maxX);
			rand.Y = FMath::RandRange(minY, maxY);
			// First check if its in bounding box if it is continue
			if (map.OutsideBoundingBoxCheck(rand)) continue;
			if (count % 10 == 0) {
				rand.X = EndPosition.X;
				rand.Y = EndPosition.Y;

				//UE_LOG(LogTemp, Display, TEXT("Sampling goal node"));
			}
			//UE_LOG(LogTemp, Display, TEXT("Sampling  node"));
			//for (int i = 0; i < CarNodes.Num(); ++i) 
			for (int i = 0; i < CarNodes.Num(); ++i) {
				if (GetCarDistance(*CarNodes[i], rand) <= GetCarDistance(*parent, rand)) {
					CarNode *nodeToTest;
					nodeToTest = CalculateCarNode(*CarNodes[i], rand);
					FVector coordinates = nodeToTest->point;
					//delete nodeToTest;
					if (!map.ObstacleCollisionCheck(coordinates)) {
						//UE_LOG(LogTemp, Display, TEXT("count: %d"), count);
						//delete NewNode;
						NewNode = CalculateCarNode(*CarNodes[i], rand);
						parent = CarNodes[i];
						foundNext = true;
					}
				}
			}
			
		}


		CarNodes.Add(new CarNode(parent, NewNode->point, NewNode->orientation));
		//Check dubins path
		
		TArray<CarNode*> trivial = LR(*NewNode, *GoalNode, map, world);
		TArray<CarNode*> trivial2 = RL(*NewNode, *GoalNode, map, world);
		TArray<CarNode*> trivial3 = CalculateTangentPoints(*NewNode, *GoalNode, map, world);
		if (trivial.Num() > trivial2.Num() && trivial2.Num() != 0) trivial = trivial2;
		if (trivial.Num() > trivial3.Num() && trivial3.Num() != 0) trivial = trivial3;

		UE_LOG(LogTemp, Display, TEXT("NR IN TRIV PATH:%d"), trivial.Num());
		if (trivial.Num() != 0) {
			trivial[0]->parent = CarNodes[CarNodes.Num()-1];
			for (int i = 0; i < trivial.Num(); ++i) {
				CarNodes.Add(new CarNode(trivial[i]->parent, trivial[i]->point, trivial[i]->orientation));
			}
			UE_LOG(LogTemp, Display, TEXT("GoalPosition:%f, %f"), GoalNode->point.X, GoalNode->point.Y);
			UE_LOG(LogTemp, Display, TEXT("foundPosition:%f, %f"), CarNodes[CarNodes.Num()-1]->point.X, CarNodes[CarNodes.Num() - 1]->point.Y);
			_CrtDumpMemoryLeaks();
			return;
		}
		//DrawDebugLine(world, CarNodes[CarNodes.Num() - 1]->point, CarNodes[CarNodes.Num() - 1]->parent->point, FColor::Red, true);
		
		UE_LOG(LogTemp, Display, TEXT("count: %d"), count);
		/*
		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			CarNodes.Add(new CarNode(parent, EndPosition, EndOrientation));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		*/
		//Draw debug line
		count++;
	}
	//UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), CarNodes.Num());
	//UE_LOG(LogTemp, Display, TEXT("Node 2 point: %f, %f"), DynamicNodes[2]->point.X, DynamicNodes[2]->point.Y);
}

