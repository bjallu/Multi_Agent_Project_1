// Fill out your copyright notice in the Description page of Project Settings.

#include "KinematicPoint.h"
#include "UnrealEngine.h"
#include "Components/SphereComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ConstructorHelpers.h"
#include "GameFramework/Pawn.h"
#include "Runtime/Json/Public/Json.h"
#include "Runtime/JsonUtilities/Public/JsonUtilities.h"
#include "Runtime/Core/Public/Templates/SharedPointer.h"
#include "Classes/Components/StaticMeshComponent.h"
#include "Runtime/Core/Public/HAL/FileManager.h"
#include "Runtime/Core/Public/Misc/Paths.h"
#include "Runtime/Core/Public/Misc/FileHelper.h"
#include <string> 
#include <list>
#include <vector>

AKinematicPoint::AKinematicPoint()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(40.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	//SphereComponent->SetSimulatePhysics(true);

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Trim.Shape_Trim"));
	if (SphereVisualAsset.Succeeded())
	{
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -40.0f));
		SphereVisual->SetWorldScale3D(FVector(0.8f));
	}
	/*
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraAttachmentArm"));
	SpringArm->SetupAttachment(RootComponent);
	SpringArm->RelativeRotation = FRotator(-45.f, 0.f, 0.f);
	SpringArm->TargetArmLength = 400.0f;
	SpringArm->bEnableCameraLag = true;
	SpringArm->bInheritYaw = 0;
	SpringArm->CameraLagSpeed = 3.0f;

	// Create a camera and attach to our spring arm
	UCameraComponent* Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("ActualCamera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
	*/
	// Take control of the default player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	MovementComponent = CreateDefaultSubobject<UKinematicPointMovementComponent>(TEXT("CustomMovementComponent"));
	MovementComponent->UpdatedComponent = RootComponent;

	//Create the NodeSelector. Used to create the graph.
	NodeSelector = NodeSelector::NodeSelector();

	double vehicle_L = 0;
	double vehicle_a_max = 0;
	double vehicle_dt = 0;
	double vehicle_omega_max = 0;
	double vehicle_t = 0;
	double vehicle_phi_max = 0;
	double vehicle_v_max = 0;
	FVector vel_goal = FVector(0.0f, 0.0f, -40.0f);
	FVector vel_start = FVector(0.0f, 0.0f, -40.0f);
	FVector pos_goal = FVector(0.0f, 0.0f, -40.0f);
	FVector pos_start = FVector(0.0f, 0.0f, -40.0f);

	m_jsonfileName = "P3";
}

// Called when the game starts or when spawned
void AKinematicPoint::BeginPlay()
{

	Super::BeginPlay();

	if (!m_jsonfileName.IsEmpty())
		ParseJson(m_jsonfileName);

}

void AKinematicPoint::DrawGraph() {
	FVector location = GetActorLocation();
	location.Z = 0;
	float x = 0;
	float y = 0;
	NodeSelector.RandomPosition(x, y);
	FVector goal = FVector(x, y, 0.f);
	NodeSelector.rrt(goal, location);
	const UWorld * world = GetWorld();
	//Draw that shit
	if (NodeSelector.nodes.Num() != 0) {
		FVector current;
		FVector next;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NodeSelector.nodes[0]->point.X, NodeSelector.nodes[0]->point.Y);
		for (int i = 1; i < NodeSelector.nodes.Num(); ++i) {
			Node* parent = NodeSelector.nodes[i]->parent;
			//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent->point.X, parent->point.Y);
			DrawDebugLine(world, NodeSelector.nodes[i]->point, parent->point, FColor::Red, true);
		}
		DrawDebugSphere(world, NodeSelector.nodes[NodeSelector.nodes.Num() - 1]->point, 5.f, 26, FColor::Blue, true);
	}
	//Move that shit
	NodeSelector.GetRrtPath(path);
	HasGoalPosition = true;
	DrawDebugLines();

}

bool AKinematicPoint::ParseJson(const FString& jsonfile)
{
	FString jsonData;
	FString jsonFileName = jsonfile + ".json";
	FString fileName = "Maps/" + jsonFileName;
	FString path = FPaths::Combine(*FPaths::GameContentDir(), *fileName);
    // Could have to use FPaths::ProjectContentDir()

	FFileHelper::LoadFileToString(jsonData, *path);
	//UE_LOG(LogTemp, Display, TEXT("%s"), *jsonData);
	//Create a pointer to hold the json serialized data
	//TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

	//Create a reader pointer to read the json data
	//const TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(jsonData);
	
	//Deserialize the json data given Reader and the actual object to deserialize
	//bool readSuccess = FJsonSerializer::Deserialize(JsonReader, JsonObject);
	//if (readSuccess)
//	{
		// Parse this shit 
		bool value = ParseMap(jsonfile, jsonFileName, jsonData);
		return value;
	//}
	//else {
		//return value;
	//}
	
}
bool AKinematicPoint::ParseMap(const FString& jsonfile, const FString& jsonFileName, const FString& jsonData)
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

	//Get the value of the json object by field name

	//public float vehicle_L, vehicle_a_max, vehicle_dt, vehicle_omega_max, vehicle_t, vehicle_phi_max, vehicle_v_max;
	//public float[] vel_goal, vel_start, pos_goal, pos_start;

	std::vector<Obstacle> vobs;

	for (auto currJsonValue = JsonObject->Values.CreateConstIterator(); currJsonValue; ++currJsonValue)
	{
		// Get the key name
		const FString Name = (*currJsonValue).Key;
		// Check if its a obstacle or the bounding box
		if (Name.Find("bounding_polygon") > -1) {
			// Create obstacle which acts as the bounding polygon
			Obstacle obs = Obstacle::Obstacle();
			// Iterate through all of the values
			//TSharedPtr<FJsonValue> Value = (*currJsonValue).Value;
			//const TArray < TSharedPtr < FJsonValue > > Value;// .AsArray();
			//std::array<std::string, 3> strarr = { "ram", "mohan", "sita" }
			//TSharedPtr<FJsonObject> jsonObj = JsonObject->GetObjectField("vehicle_L");
			TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
			for (int32 i = 0; i < objArray.Num(); i++)
			{
				TArray<TSharedPtr<FJsonValue>> Obstacle_Coordinates = objArray[i]->AsArray();
				double x = Obstacle_Coordinates[0]->AsNumber();
				double y = Obstacle_Coordinates[1]->AsNumber();
				obs.AddObstaclePoint(x, y);
			}
			vobs.push_back(obs);
		}
		else if (Name.Find("obstacle") > -1) {
			Obstacle obs = Obstacle::Obstacle();
			// Iterate through all of the values
			//TSharedPtr<FJsonValue> Value = (*currJsonValue).Value;
			//const TArray < TSharedPtr < FJsonValue > > Value;// .AsArray();
			//std::array<std::string, 3> strarr = { "ram", "mohan", "sita" }
			//TSharedPtr<FJsonObject> jsonObj = JsonObject->GetObjectField("vehicle_L");
			TArray<TSharedPtr<FJsonValue>> objArray = JsonObject->GetArrayField(Name);
			for (int32 i = 0; i < objArray.Num(); i++)
			{
				TArray<TSharedPtr<FJsonValue>> Obstacle_Coordinates = objArray[i]->AsArray();
				double x = Obstacle_Coordinates[0]->AsNumber();
				double y = Obstacle_Coordinates[1]->AsNumber();
				obs.AddObstaclePoint(x, y);
			}
			vobs.push_back(obs);
		}
		// Now only values which we need to pair up no more obstacles
		// remember origin coordinates
		else {
			if (Name == "vehicle_L"){
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
			/*
			for (int32 Index = 0; Index != Value.; ++Index)
			{
				JoinedStr += StrArr[Index];
				JoinedStr += TEXT(" ");
			}
			for (const std::string& str : strarr) {
				obs.AddObstaclePoint()
				listbox.items.add(str);
			}
		}
		// Otherwise just get the parameter values 

		// Get the value as a FJsonValue object
		TSharedPtr< FJsonValue > Value = (*currJsonValue).Value;

		UE_LOG(LogTemp, Display, TEXT("parameter %s "), *Name);
		// Do your stuff with crazy casting and other questionable rituals
	}
	/*
	string[] json = jsonString.Split('\n');
	obstacles = new List<Polygon>();
	for (int i = 0; i < json.Length; i++) {
		string obstacle = json[i];
		if ( (obstacle.find("bounding_polygon") != std::string::npos) || (obstacle.find("obstacle" != std::string::npos)){
			//Polygon poly = new Polygon(json[i]); our obstacle class do when we merge
			for (int j = i + 1; j < obstacle.Length - 1; j += 4) {
				if (json[j].Contains("]") && json[j - 1].Contains("]")) {
					break;
				}
				if (json[j].Contains("[")) {
					string x_cor = json[j + 1].Trim(' ').Trim(',').Trim(',').Trim('\r').Trim(',');
					string y_cor = json[j + 2].Trim(' ').Trim(',');
					double x_coordinate = double.Parse(x_cor, CultureInfo.InvariantCulture);
					double y_coordinate = double.Parse(y_cor, CultureInfo.InvariantCulture);
					poly.addCorner(x_coordinate, y_coordinate);
				}
			}
			obstacles.Add(poly);
		}
	}
	string str("There are two needles in this haystack.");
	string str2("needle");

	if (str.find(str2) != string::npos) {
		//.. found.
	}
	TSharedPtr<FJsonObject> jsonObj = JsonObject->GetObjectField("vehicle_L");
	TArray<TSharedPtr<FJsonValue>> objArray = jsonObj->GetArrayField("rows");

	for (int32 i = 0; i < objArray.Num(); i++)
	{

		TArray<TSharedPtr<FJsonValue>> height = objArray[i]->AsArray();
		FString name = height[0]->AsString();

		UE_LOG(LogTemp, Warning, TEXT("Value I'm looking for %s"), *name);
	}
	/*
	TArray< TSharedPtr<FJsonValue> > ParsedTableRows = JsonObject->GetArrayField(arrayFieldName);

	// Iterate over rows
	for (int32 RowIdx = 0; RowIdx < ParsedTableRows.Num(); ++RowIdx)
	{
		const TSharedPtr<FJsonValue>& ParsedTableRowValue = ParsedTableRows[RowIdx];
		TSharedPtr<FJsonObject> ParsedTableRowObject = ParsedTableRowValue->AsObject();
		if (!ParsedTableRowObject.IsValid())
		{
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red,
				FString::Printf(TEXT("Row '%d' is not a valid JSON object."),
					RowIdx));
			continue;
		}

		// read row. if it return false, continue
		if (!ReadPhraseRow(ParsedTableRowObject.ToSharedRef(), RowIdx))
			continue;
		*/
	/*
	// Empty existing data
	m_PhraseDataTable->EmptyTable();

	// Array Field Name - The first array field's name
	// should be same as character
	FString arrayFieldName = jsonfile;

	// Parse by my rule
	TArray< TSharedPtr<FJsonValue> > ParsedTableRows = JsonObject->GetArrayField(arrayFieldName);

	// Iterate over rows
	for (int32 RowIdx = 0; RowIdx < ParsedTableRows.Num(); ++RowIdx)
	{
		const TSharedPtr<FJsonValue>& ParsedTableRowValue = ParsedTableRows[RowIdx];
		TSharedPtr<FJsonObject> ParsedTableRowObject = ParsedTableRowValue->AsObject();
		if (!ParsedTableRowObject.IsValid())
		{
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red,
				FString::Printf(TEXT("Row '%d' is not a valid JSON object."),
					RowIdx));
			continue;
		}

		// read row. if it return false, continue
		if (!ReadPhraseRow(ParsedTableRowObject.ToSharedRef(), RowIdx))
			continue;
	}

	    public float vehicle_L, vehicle_a_max, vehicle_dt, vehicle_omega_max, vehicle_t, vehicle_phi_max, vehicle_v_max;
    public float[] vel_goal, vel_start, pos_goal, pos_start;
    public List<Polygon> obstacles;

    public static Problem Import(string filePath) {
        StreamReader reader = new StreamReader(filePath);
        string json = reader.ReadToEnd();
        reader.Close();
        Problem map = JsonUtility.FromJson<Problem>(json);
        map.read_polygons(json);
        return map;
    }

    private void read_polygons(string jsonString) {
        string[] json = jsonString.Split('\n');
        obstacles = new List<Polygon>();
        for (int i = 0; i < json.Length; i++) {
            if (json[i].Contains("bounding_polygon") || json[i].Contains("obstacle")) {
                Polygon poly = new Polygon(json[i]);
                for (int j = i + 1; j < json.Length - 1; j += 4) {
                    if (json[j].Contains("]") && json[j - 1].Contains("]")) {
                        break;
                    }
                    if (json[j].Contains("[")) {
                        string x_cor = json[j + 1].Trim(' ').Trim(',').Trim(',').Trim('\r').Trim(',');
                        string y_cor = json[j + 2].Trim(' ').Trim(',');
                        double x_coordinate = double.Parse(x_cor, CultureInfo.InvariantCulture);
                        double y_coordinate = double.Parse(y_cor, CultureInfo.InvariantCulture);
                        poly.addCorner(x_coordinate, y_coordinate);
                    }
                }
                obstacles.Add(poly);
            }
        }

    }

	// Modify the datatable
	m_PhraseDataTable->Modify(true);
	*/

	DrawMap(vobs);

	return true;
}


// Called every frame
void AKinematicPoint::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (HasGoalPosition) {
		//Calculate path
		MoveToPosition(path[0].X, path[0].Y);
		FVector XYLoc = GetActorLocation();
		XYLoc.Z = 0.f;
		//path[0].Z = 0.f;
		//UE_LOG(LogTemp, Display, TEXT("Distance To Goal: %f"), *(XYLoc - GoalPosition).SizeSquared());
		//Check if GoalPosition is reached
		if ((XYLoc - path[0]).SizeSquared() <= 1.f) {
			// goal reached, check if path is not empty
			path.RemoveAt(0);
			if (path.Num() == 0) {
				HasGoalPosition = false;
				FlushPersistentDebugLines(GetWorld());
			}
				

		}
		else
			MovementComponent->AddInputVector(GetActorForwardVector());
	}
	

}


void AKinematicPoint::DrawMap(std::vector<Obstacle> obs) {
	const UWorld *world = GetWorld();
	for (int i = 0; i < obs.size(); ++i){
		Obstacle obstocheck = obs[i];
		for (int j = 0; j < obstocheck.points.size(); ++j) {
			// Draw from the last to the first 
			if(j==(obstocheck.points.size()-1)){
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], GetActorLocation().Z), FVector(obstocheck.points[0][2], obstocheck.points[0][3], GetActorLocation().Z), FColor::Emerald, true);
			}
			//Otherwise we always draw to the next one
			else {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], GetActorLocation().Z), FVector(obstocheck.points[j+1][2], obstocheck.points[j+1][3], GetActorLocation().Z), FColor::Emerald, true);
			}
		}
	}
}

void AKinematicPoint::DrawDebugLines() {
	if (path.Num() != 0) {
		const UWorld *world = GetWorld();
		FVector current;
		FVector next;
		DrawDebugLine(world, GetActorLocation(), FVector(path[0].X, path[0].Y, GetActorLocation().Z), FColor::Emerald, true);
		for (int i = 0; i < path.Num() - 1; ++i) {
			current = FVector(path[i].X, path[i].Y, GetActorLocation().Z);
			next = FVector(path[i + 1].X, path[i + 1].Y, GetActorLocation().Z);
			DrawDebugLine(world, current, next, FColor::Emerald, true);
		}
	}
}
// Called to bind functionality to input
void AKinematicPoint::SetupPlayerInputComponent(UInputComponent* InputComponent)
{
	//Super::SetupPlayerInputComponent(InputComponent);

	InputComponent->BindAxis("MoveForward", this, &AKinematicPoint::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &AKinematicPoint::MoveRight);
	//InputComponent->BindAxis("Turn", this, &AKinematicPoint::Turn);
	InputComponent->BindAction("RandomDirection", IE_Pressed, this, &AKinematicPoint::RandomTurn);
	InputComponent->BindAction("RandomPosition", IE_Pressed, this, &AKinematicPoint::RandomPosition);
	InputComponent->BindAction("RandomPath", IE_Pressed, this, &AKinematicPoint::GetPath);
	InputComponent->BindAction("DrawGraph", IE_Pressed, this, &AKinematicPoint::DrawGraph);


}

UPawnMovementComponent* AKinematicPoint::GetMovementComponent() const
{
	return MovementComponent;
}

void AKinematicPoint::MoveForward(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{

		MovementComponent->AddInputVector(GetActorForwardVector() * AxisValue);
	}

}

void AKinematicPoint::MoveRight(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += AxisValue;
		SetActorRotation(NewRotation);
	}


}

void AKinematicPoint::Turn(float AxisValue)
{
	FRotator NewRotation = SpringArm->RelativeRotation;
	NewRotation.Yaw += AxisValue;
	SpringArm->SetRelativeRotation(NewRotation);
}

void AKinematicPoint::RandomTurn() {
	FRotator NewRotation = GetActorRotation();
	NewRotation.Yaw += rand();
	SetActorRotation(NewRotation);
}

void AKinematicPoint::RandomPosition() {
	float x;
	float y;
	this->NodeSelector.RandomPosition(x,y);
	//UE_LOG(LogTemp,Display, TEXT("X: %f Y: %f"), x, y);
	path.Add(FVector(x, y, 0.f));
	HasGoalPosition = true;
	DrawDebugLines();
}

void AKinematicPoint::GetPath() {
	NodeSelector.GetPath(path);
	HasGoalPosition = true;
	DrawDebugLines();
	
}

void AKinematicPoint::MoveToPosition(float x, float y) {
	//Calculate rotation
	FVector MoveTo = FVector(x, y, 0);
	FVector Forward =   MoveTo-GetActorLocation();
	Forward.Z = 0;
	FRotator Rotation = FRotationMatrix::MakeFromX(Forward).Rotator();
	SetActorRotation(Rotation);
}
