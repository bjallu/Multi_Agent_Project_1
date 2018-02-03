#pragma once
#include "CoreMinimal.h"
#include "Obstacle.h"
#include <vector>


class MapFunctions
{
public:
	// Bounding box obstacle
	Obstacle bounding_box;
	// Rest of the obstacles
	std::vector<Obstacle> obstacles;
	FString m_jsonfileName;
	float z;
	double vehicle_L;
	double vehicle_a_max;
	double vehicle_dt;
	double vehicle_omega_max;
	double vehicle_t;
	double vehicle_phi_max;
	double vehicle_v_max;
	FVector vel_goal;
	FVector vel_start;
	FVector pos_goal;
	FVector pos_start;
	double global_z_offset;
	MapFunctions();
	~MapFunctions();

public:
	// Our method of drawing the "map" provided
	// Our json parser for the P1-P3 files
	bool ParseJson(const FString& jsonfile);
	bool ParseMap(const FString& jsonfile, const FString& jsonFileName, const FString& jsonData);
	// Find Min and Max x and y coordinates for all objects
	void FindMin(Obstacle& obs, const float& x, const float& y);
	void FindMax(Obstacle& obs, const float& x, const float& y);
	// Collision Functionality
	bool OutsideBoundingBoxCheck(const FVector& pointToCheck);
	bool ObstacleCollisionCheck(const FVector& pointToCheck);
	bool CollisionCheck(const FVector&, const Obstacle& obs);
	bool isInside(Obstacle obs, int n, FVector p);
	bool onSegment(FVector p, FVector q, FVector r);
	int orientation(FVector p, FVector q, FVector r);
	bool doIntersect(FVector p1, FVector q1, FVector p2, FVector q2);
};

