// Fill out your copyright notice in the Description page of Project Settings.

#include "NodeSelector.h"



NodeSelector::NodeSelector()
{
	XBound = 30.f;
	YBound = 30.f;
	PathSize = 3;
	NumNodes = 3000;
	GoalRadius = 0.3f;
	nodes = TArray<Node*>();
	CarNodes = TArray<CarNode*>();
	DynamicNodes = TArray<DynamicNode*>();
	obstacles = TArray<Obstacle>();
	StepSize = 20;
	TimeStep = 0.1;
	VehicleLength = 2.0;
	Velocity = 1.0;
	MaxTurnSpeed = 1.0;
	Acceleration = 1.3;
	MaxTurnAngle = 0.6;
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

void NodeSelector::getCarRrtPath(TArray<CarNode*>& vectors) {
	vectors.Empty();
	//nodes[0] is startposition so shouldnt include it
	CarNode* CurrentNode = CarNodes[CarNodes.Num() - 1];
	while (CurrentNode->point != CarNodes[0]->point) {
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
		//UE_LOG(LogTemp, Display, TEXT("CurrentNode x, y : %f, %f"), CurrentNode->point.X, CurrentNode->point.Y);
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
	direction *= Acceleration * TimeStep;     //Set to max acceleration
	FVector NewVelocity = currentVelocity + direction;
	//NewVelocity = NewVelocity * Velocity / NewVelocity.Size(); //Set length to velocity
	if (NewVelocity.Size() > Velocity) {
		NewVelocity /= currentVelocity.Size(); //Normalize
		NewVelocity *= Velocity;			//Set to max velocity
												//UE_LOG(LogTemp, Display, TEXT("New Velocity: %f"), currentVelocity.Size());
	}
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
	DynamicNode* NewNode = parent;
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
					NewNode = CalculateDynamicPointNode(*DynamicNodes[i], rand);
					parent = DynamicNodes[i];
					foundNext = true;
				}
			}
		}
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);

		DynamicNodes.Add(new DynamicNode(parent, NewNode->point,NewNode->Velocity));

		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			DynamicNodes.Add(new DynamicNode(parent, EndPosition, EndVelocity));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
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

std::vector<std::pair<FVector,FVector>> NodeSelector::TangentLines(FVector c1, FVector c2, float radc1, float radc2) {
	float d_sq = pow(c2.X-c1.X,2)+pow(c2.Y-c1.Y,2);
	std::vector<std::pair<FVector, FVector>> returnVec;
	if (d_sq < (radc1 - radc2)*(radc1 - radc2)) {
		//we may have a problem, the circles are either intersecting, one is within the other, but still tangent
		//at one point, or one is completely in the other. We only have a problem if one is within the other, but
		//not tangent to it anywhere
		if (d_sq != std::fmaxf(radc1, radc2) && d_sq < std::fmaxf(radc1, radc2)){
			return returnVec;
		}//else they are intersecting or one is within the other, but still tangent to it
		 //in the other two cases, either 1 or 2 external tangent lines remain, but there are no internal tangent
		 //lines
	}

	float d = sqrtf(d_sq);
	float vx = (c2.X - c1.X) / d;
	float vy = (c2.Y - c1.Y) / d;
	for (int sign1 = +1; sign1 >= -1; sign1 -= 2) {
		float c = (radc1 - sign1 * radc2) / d;
		if (c*c > 1.0) continue; //want to be subtracting small from large, not adding
		double h = sqrt(std::fmaxf(0.0, 1.0 - c * c));

		for (int sign2 = +1; sign2 >= -1; sign2 -= 2) {
			float nx = vx * c - sign2 * h * vy;
			float ny = vy * c + sign2 * h * vx;
			returnVec.push_back(std::make_pair((FVector(c1.X + radc1 * nx, c1.Y + radc1 * ny,0.0f)),
				(FVector(c2.X + sign1 * radc2 * nx, c2.Y + sign1 * radc2 * ny,0.0f))));
		}
	}
	
	return std::vector<std::pair<FVector, FVector>>();
}

float NodeSelector::ArcLength(FVector center, FVector left, FVector right, float radius, bool isleft) {
	FVector vec1, vec2;
	vec1 = left - center;
	vec2 = right - center;
	float theta = atan2(vec2.Y, vec2.X) - atan2(vec1.Y, vec1.X);
	if (theta < -1e-6 && isleft)
		theta += 2.0*PI;
	else if (theta > 1e-6 && !isleft)
		theta -= 2.0 * PI;
	return fabs(theta*radius);
}

TArray<CarNode*> NodeSelector::LSR(std::vector<std::pair<FVector, FVector>>& _LRTangents, const FVector agentleft, float radleft, const FVector goalright, float radright, FVector currPos,FVector goalPos,const CarNode&n1) {
	TArray<CarNode*> next;
	float arcL1, arcL2, arcL3;
	if (_LRTangents.size() > 3) {
		//CALCULATE LEFT CIRCLE
		arcL1 = ArcLength(agentleft, currPos, _LRTangents[3].first, radleft, true);
		float timesteps = arcL1 / (Velocity * TimeStep);
		float delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
		float orientation = acos(GetCosAngle(FVector(1, 0, 0), n1.orientation));
		orientation = delta + orientation;
		if (n1.orientation.Y < 0)
			orientation = -orientation;
		FVector newOrientation = FVector(cos(orientation), sin(orientation), n1.point.Z);
		float newX = Velocity * cos(orientation)*TimeStep;
		float newY = Velocity * sin(orientation)*TimeStep;
		FVector newPosition = FVector(n1.point.X + newX, n1.point.Y + newY, n1.point.Z);
		next.Add(new CarNode(n1, newPosition, newOrientation));
		for (int i = 1; i < timesteps; ++i) {
			orientation = acos(GetCosAngle(FVector(1, 0, 0), next[i-1]->orientation));
			if (next[i - 1]->orientation.Y < 0)
				orientation = -orientation;
			orientation = delta + orientation; //THIS MIGHT BE TURN RIGHT
			newOrientation = FVector(cos(orientation), sin(orientation), next[i - 1]->point.Z);
			newX = Velocity * cos(orientation)*TimeStep;
			newY = Velocity * sin(orientation)*TimeStep;
			newPosition = FVector(next[i - 1]->point.X + newX, next[i - 1]->point.Y + newY, next[i - 1]->point.Z);
			next.Add(new CarNode(next[i - 1], newPosition, newOrientation));	
		}
		//CALCULATE STRAIGHT LINE
		arcL2 = (_LRTangents[3].first - _LRTangents[3].second).Size();
		timesteps = arcL2 / (Velocity * TimeStep);
		newX = Velocity * cos(orientation)*TimeStep;
		newY = Velocity * sin(orientation)*TimeStep;
		newPosition = FVector(next[next.Num()-1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
		next.Add(new CarNode(n1, newPosition, newOrientation));
		for (int i = 1; i < timesteps; ++i) {
			newX = Velocity * cos(orientation)*TimeStep;
			newY = Velocity * sin(orientation)*TimeStep;
			newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
			next.Add(new CarNode(next[next.Num() - 1], newPosition, newOrientation));
		}
		//CALCULATE RIGHT CIRCLE
		arcL3 = ArcLength(goalright, _LRTangents[3].second,goalPos, radleft, false);
		timesteps = arcL3 / (Velocity * TimeStep);
		delta = (Velocity / VehicleLength)*tan(MaxTurnAngle)*TimeStep;
		orientation = acos(GetCosAngle(FVector(1, 0, 0), next[next.Num() - 1]->orientation));
		if (next[next.Num() - 1]->orientation.Y < 0)
			orientation = -orientation;
		orientation = delta + orientation;
		newOrientation = FVector(cos(orientation), sin(orientation), next[next.Num() - 1]->point.Z);
		newX = Velocity * cos(orientation)*TimeStep;
		newY = Velocity * sin(orientation)*TimeStep;
		newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
		next.Add(new CarNode(n1, newPosition, newOrientation));
		for (int i = 1; i < timesteps; ++i) {
			orientation = acos(GetCosAngle(FVector(1, 0, 0), next[next.Num() - 1]->orientation));
			if (next[next.Num() - 1]->orientation.Y < 0)
				orientation = -orientation;
			orientation = delta - orientation; //THIS MIGHT BE TURN RIGHT
			newOrientation = FVector(cos(orientation), sin(orientation), next[next.Num() - 1]->point.Z);
			newX = Velocity * cos(orientation)*TimeStep;
			newY = Velocity * sin(orientation)*TimeStep;
			newPosition = FVector(next[next.Num() - 1]->point.X + newX, next[next.Num() - 1]->point.Y + newY, next[next.Num() - 1]->point.Z);
			next.Add(new CarNode(next[next.Num() - 1], newPosition, newOrientation));
		}
	}
	return next;
}

TArray<CarNode*> NodeSelector::CalculateTangentPoints(CarNode& n1, CarNode& n2) {
	float r1 = VehicleLength / tan(MaxTurnAngle);
	float r2 = r1;
	FVector cross = FVector::CrossProduct(n1.orientation, FVector(0, 0, 1));
	FVector fromLeft = n1.point + ((cross / cross.Size()))*r1; //Left Circle at current position
	FVector fromRight = n1.point - ((cross / cross.Size()))*r1;
	cross = FVector::CrossProduct(n2.orientation, FVector(0, 0, 1));
	FVector toLeft = n2.point + ((cross / cross.Size()))*r2;
	FVector toRight = n2.point - ((cross / cross.Size()))*r2;
	TArray<CarNode*> next;
	std::vector<std::pair<FVector, FVector>> _LSRTangents = TangentLines(fromLeft, toRight, r1,r2);
	next = LSR(_LSRTangents, fromLeft, r1, toRight, r2, n1.point, n2.point, n1);
	// Check if collides, check if current orientation is correct get smallest path of those.
	for (int i = 0; i < next.Num(); ++i) {
		//check collides
		if(map.ObstacleCollisionCheck(next[i]))

	}

	


}

CarNode* NodeSelector::GetDubinsPath(const CarNode&n1, const CarNode&n2) {
	float r1 = VehicleLength/tan(MaxTurnAngle);
	float r2 = r1;
	
	
	
	


	return new CarNode(n1);
}


void NodeSelector::carRrt(FVector EndPosition, FVector StartPosition, FVector StartOrientation, FVector EndOrientation) {
	CarNodes.Empty();
	//Create startnode
	CarNode* StartNode = new CarNode(StartPosition, StartOrientation);
	CarNodes.Add(StartNode);
	int count = 0;
	bool foundNext = false;
	float x = 0;
	float y = 0;
	FVector rand = FVector(x, y, StartPosition.Z);
	CarNode* parent = CarNodes[0];
	CarNode* NewNode = parent;
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
			for (int i = 0; i < CarNodes.Num(); ++i) {
				if (PointDistance(CarNodes[i]->point, rand) <= PointDistance(parent->point, rand)) {
					NewNode = CalculateCarNode(*CarNodes[i], rand);
					parent = CarNodes[i];
					foundNext = true;
				}
			}
		}
		//NewNode.Z = 70.f;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NewNode.X, NewNode.Y);

		CarNodes.Add(new CarNode(parent, NewNode->point, NewNode->orientation));

		if (PointDistance(NewNode->point, EndPosition)<GoalRadius) {
			CarNodes.Add(new CarNode(parent, EndPosition, EndOrientation));
			UE_LOG(LogTemp, Display, TEXT("FOUND GOAL"));
			count = NumNodes;
		}
		//Draw debug line
		count++;
	}
	UE_LOG(LogTemp, Display, TEXT("nr of nodes: %d"), CarNodes.Num());
	//UE_LOG(LogTemp, Display, TEXT("Node 2 point: %f, %f"), DynamicNodes[2]->point.X, DynamicNodes[2]->point.Y);
}
