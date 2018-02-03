#include "MapFunctions.h"
#include "UnrealEngine.h"
#include "ConstructorHelpers.h"
#include "Runtime/Json/Public/Json.h"
#include "Runtime/JsonUtilities/Public/JsonUtilities.h"
#include "Runtime/Core/Public/Templates/SharedPointer.h"
#include "Runtime/Core/Public/HAL/FileManager.h"
#include "Runtime/Core/Public/Misc/Paths.h"
#include "Runtime/Core/Public/Misc/FileHelper.h"
#include <string> 
#include <list>
#include <vector>
#include <iostream>
#include <algorithm>    // std::max/min
using namespace std;

// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000


MapFunctions::MapFunctions()
{
	// Bounding box obstacle
	Obstacle bounding_box = Obstacle::Obstacle();
	// Rest of the obstacles
	std::vector<Obstacle> obstacles;
	z = 32.0f;
	// Velocity values
	vehicle_L = 0;
	vehicle_a_max = 0;
	vehicle_dt = 0;
	vehicle_omega_max = 0;
	vehicle_t = 0;
	vehicle_phi_max = 0;
	vehicle_v_max = 0;
	// Positional values
	vel_goal = FVector(0.0f, 0.0f, z);
	vel_start = FVector(0.0f, 0.0f, z);
	pos_goal = FVector(0.0f, 0.0f, z);
	pos_start = FVector(0.0f, 0.0f, z);
	// Global offsets
	double global_z_offset = z;
	// The file name
	m_jsonfileName = "P3";
}

void MapFunctions::FindMin(Obstacle& obs, const float& x, const float& y) {
	if (x < obs.minX) {
		obs.minX = x;
	}
	if (y < obs.minY) {
		obs.minY = y;
	}
}

void MapFunctions::FindMax(Obstacle& obs, const float& x, const float& y) {
	if (x > obs.maxX) {
		obs.maxX = x;
	}
	if (y > obs.maxY) {
		obs.maxY = y;
	}
}

bool MapFunctions::ParseJson(const FString& jsonfile)
{
	FString jsonData;
	FString jsonFileName = jsonfile + ".json";
	FString fileName = "Maps/" + jsonFileName;
	FString path = FPaths::Combine(*FPaths::GameContentDir(), *fileName);
	// Could have to use FPaths::ProjectContentDir()

	FFileHelper::LoadFileToString(jsonData, *path);
	// Parse this shit 
	bool value = ParseMap(jsonfile, jsonFileName, jsonData);
	return value;
}

bool MapFunctions::ParseMap(const FString& jsonfile, const FString& jsonFileName, const FString& jsonData)
{
	//Create a pointer to hold the json serialized data
	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

	//Create a reader pointer to read the json data
	const TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(jsonData);

	// Process of Deserializing
	bool readSuccess = FJsonSerializer::Deserialize(JsonReader, JsonObject);
	if (!readSuccess)
	{
		UE_LOG(LogTemp, Display, TEXT("Couldnt serialize (sadface)"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, jsonFileName);
		return false;
	}
	// Else we start our parsing
	for (auto currJsonValue = JsonObject->Values.CreateConstIterator(); currJsonValue; ++currJsonValue)
	{
		// Get the key name
		const FString Name = (*currJsonValue).Key;
		// Check if its a obstacle or the bounding box
		if (Name.Find("bounding_polygon") > -1) {
			// Create obstacle which acts as the bounding polygon
			Obstacle obs = Obstacle::Obstacle();
			TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
			for (int32 i = 0; i < objArray.Num(); i++)
			{
				TArray<TSharedPtr<FJsonValue>> Obstacle_Coordinates = objArray[i]->AsArray();
				double x = Obstacle_Coordinates[0]->AsNumber();
				double y = Obstacle_Coordinates[1]->AsNumber();
				obs.AddObstaclePoint(x, y);
				FindMax(obs, x, y);
				FindMin(obs, x, y);
			}
			bounding_box = obs;
		}
		else if (Name.Find("obstacle") > -1) {
			Obstacle obs =  Obstacle::Obstacle();
			TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
			for (int32 i = 0; i < objArray.Num(); i++)
			{
				TArray<TSharedPtr<FJsonValue>> Obstacle_Coordinates = objArray[i]->AsArray();
				double x = Obstacle_Coordinates[0]->AsNumber();
				double y = Obstacle_Coordinates[1]->AsNumber();
				obs.AddObstaclePoint(x, y);
				FindMax(obs, x, y);
				FindMin(obs, x, y);
			}
			obstacles.push_back(obs);
		}
		else {
			if (Name == "vehicle_L") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_L = objvalue;
			}
			if (Name == "vehicle_a_max") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_a_max = objvalue;
			}
			if (Name == "vehicle_dt") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_dt = objvalue;
			}
			if (Name == "vehicle_omega_max") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_omega_max = objvalue;
			}
			if (Name == "vehicle_t") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_t = objvalue;
			}
			if (Name == "vehicle_phi_max") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_phi_max = objvalue;
			}
			if (Name == "vehicle_v_max") {
				double objvalue = JsonObject->GetNumberField(Name);
				vehicle_v_max = objvalue;
			}
			if (Name == "vel_goal") {
				TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
				double x = objArray[0]->AsNumber();
				double y = objArray[1]->AsNumber();
				vel_goal = FVector(x, y, -40.f);
			}
			if (Name == "vel_start") {
				TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
				double x = objArray[0]->AsNumber();
				double y = objArray[1]->AsNumber();
				vel_start = FVector(x, y, -40.f);
			}
			if (Name == "pos_goal") {
				TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
				double x = objArray[0]->AsNumber();
				double y = objArray[1]->AsNumber();
				pos_goal = FVector(x, y, -40.0f);
			}
			if (Name == "pos_start") {
				TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
				double x = objArray[0]->AsNumber();
				double y = objArray[1]->AsNumber();
				pos_start = FVector(x, y, -40.0f);
			}
		}
	}
	// put all the drawing logic in the kynamatic point, dynamic point etc etc. 
	//DrawMap(bounding_box);
	//DrawMap(obstacles);
	return true;
}

bool MapFunctions::OutsideBoundingBoxCheck(const FVector& pointToCheck) {
	Obstacle obs = this->bounding_box;
	// First check to try and speed this up
	if (pointToCheck.X < obs.minX || pointToCheck.X > obs.maxX || pointToCheck.Y < obs.minY || pointToCheck.Y > obs.maxY) {
		// We're outside the polygon! so we return true
		return true;
	}
	// Else We go into a more detailed  check for harder cases

	//
	//true if above conditions are met odd number of times and false otherwise.But what does that mean ? !?
	int i = 0;
	int j = 0; 
	int c = 0;
	for (int i = 0, j = obs.points.size() - 1; i < obs.points.size(); j = i++) {
		if (((obs.points[i][3] > pointToCheck.Y) != (obs.points[j][3] > pointToCheck.Y)) &&
			(pointToCheck.X < (obs.points[j][2] - obs.points[i][2]) * (pointToCheck.Y - obs.points[i][3]) / (obs.points[j][3] - obs.points[i][3]) + obs.points[i][2]))
			c = !c;
	}
	if (c % 2 == 0)
		return true;
	else
		return false;
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool MapFunctions::isInside(Obstacle obs, int n, FVector p){
	// There must be at least 3 vertices in polygon[]
	if (n < 3)  return false;

	// Create a point for line segment from p to infinite
	//float[] extreme;
	//extreme = { INF, p.y };
	FVector extreme = FVector(INF, p.Y, -40.0f);

	// Count intersections of the above line with sides of polygon
	int count = 0;
	int i = 0;
	do
	{
		int next = (i + 1) % n;

		std::vector<int> obs_i = obs.points[i];
		FVector obs__i = FVector(obs_i[2], obs_i[3], -40.0f);
		std::vector<int> obs_next = obs.points[next];
		FVector obs__next = FVector(obs_next[2], obs_next[3], -40.0f);

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(obs__i, obs__next, p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(obs__i, p, obs__next) == 0)
				return onSegment(obs__i, p, obs__next);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count & 1;  // Same as (count%2 == 1)
}

bool MapFunctions::ObstacleCollisionCheck(const FVector& pointToCheck) {
	//std::vector<Obstacle*> obs = this->obstacles;
	std::vector<Obstacle> obs = obstacles;
	for (int k = 0; k < obs.size(); ++k) {
		Obstacle obstocheck = obs[k];
		int nvert = obstocheck.points.size();
		if (isInside(obstocheck, nvert, pointToCheck)) {
			return true;
		}
		/*
		double testx = pointToCheck.X;
		double testy = pointToCheck.Y;
		for (int k = 0; k < obs.size(); ++k) {
		bool inside = false;
		int i = 0;
		int j = 0;
		//Obstacle* obstocheck = obs[k];
		Obstacle obstocheck = obs[k];
		if (pointToCheck.X > obstocheck.minX && pointToCheck.X < obstocheck.maxX && pointToCheck.Y > obstocheck.minY && pointToCheck.Y < obstocheck.maxY) {
		// We're inside the polygon! so we return true
		return true;
		}
		int nvert = obstocheck.points.size();
		for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((obstocheck.points[i][3] > testy) != (obstocheck.points[j][3] > testy)) &&
		(testx < (obstocheck.points[j][2] - obstocheck.points[i][2]) * (testy - obstocheck.points[i][3]) / (obstocheck.points[j][3] - obstocheck.points[i][3]) + obstocheck.points[i][2]))
		inside = !inside;
		}
		UE_LOG(LogTemp, Display, TEXT("%c"), inside);
		if (inside)return true;
		}
		return false;
		*/
	}
	return false;
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool MapFunctions::onSegment(FVector p, FVector q, FVector r){
	if (q.X <= max(p.X, r.X) && q.X >= min(p.X, r.X) &&
		q.Y <= max(p.Y, r.Y) && q.Y >= min(p.Y, r.Y)) {
		return true;
	}
	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int MapFunctions::orientation(FVector p, FVector q, FVector r)
{
	int val = (q.Y - p.Y) * (r.X - q.X) -
		(q.X - p.X) * (r.Y - q.Y);

	if (val == 0) return 0;  // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool MapFunctions::doIntersect(FVector p1, FVector q1, FVector p2, FVector q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

/*
void MapFunctions::DrawMap(std::vector<Obstacle> obs) {
	const UWorld *world = GetWorld();
	for (int i = 0; i < obs.size(); ++i) {
		Obstacle obstocheck = obs[i];
		for (int j = 0; j < obstocheck.points.size(); ++j) {
			// Draw from the last to the first 
			if (j == (obstocheck.points.size() - 1)) {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], global_z_offset), FVector(obstocheck.points[0][2], obstocheck.points[0][3], global_z_offset), FColor::Emerald, true);
			}
			//Otherwise we always draw to the next one
			else {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], global_z_offset), FVector(obstocheck.points[j + 1][2], obstocheck.points[j + 1][3], global_z_offset), FColor::Emerald, true);
			}
		}
	}
}
*/
MapFunctions::~MapFunctions()
{
}
