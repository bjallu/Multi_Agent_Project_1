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


MapFunctions::MapFunctions()
{
	// Bounding box obstacle
	Obstacle bounding_box = Obstacle::Obstacle();
	// Rest of the obstacles
	std::vector<Obstacle> obstacles;
	// Velocity values
	double vehicle_L = 0;
	double vehicle_a_max = 0;
	double vehicle_dt = 0;
	double vehicle_omega_max = 0;
	double vehicle_t = 0;
	double vehicle_phi_max = 0;
	double vehicle_v_max = 0;
	// Positional values
	FVector vel_goal = FVector(0.0f, 0.0f, -40.0f);
	FVector vel_start = FVector(0.0f, 0.0f, -40.0f);
	FVector pos_goal = FVector(0.0f, 0.0f, -40.0f);
	FVector pos_start = FVector(0.0f, 0.0f, -40.0f);
	// Global offsets
	double global_z_offset = -40.0f;
	// The file name
	m_jsonfileName = "P3";
}

void MapFunctions::FindMin(Obstacle& obs, const float& x, const float& y) {
	if (x < obs.minX) {
		obs.minX = x;
	}
	if (y < obs.minY) {
		obs.minY;
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

bool MapFunctions::ObstacleCollisionCheck(const FVector& pointToCheck) {
	std::vector<Obstacle> obs = this->obstacles;
	for (int i = 0; i < obs.size(); ++i) {
		if (CollisionCheck(pointToCheck, obs[i])) {
			return true;
		}
	}
	return false;
}

bool MapFunctions::CollisionCheck(const FVector& pointToCheck, const Obstacle& obs) {
	int i = 0; 
	int j = 0; 
	int c = 0;
	for (int i = 0, j = obs.points.size() - 1; i < obs.points.size(); j = i++) {
		if (((obs.points[i][3] > pointToCheck.Y) != (obs.points[j][3] > pointToCheck.Y)) &&
			(pointToCheck.X < (obs.points[j][2] - obs.points[i][2]) * (pointToCheck.Y - obs.points[i][3]) / (obs.points[j][3] - obs.points[i][3]) + obs.points[i][2]))
			c = !c;
	}
	if (c == 1)
		return true;
	return false;
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
