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
#include <MapFunctions.h>

AKinematicPoint::AKinematicPoint()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(40.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	//SphereComponent->SetSimulatePhysics(true);
	/*
	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Trim.Shape_Trim"));
	if (SphereVisualAsset.Succeeded())
	{
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -40.0f));
		SphereVisual->SetWorldScale3D(FVector(1.f));
	}
	
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
	map = MapFunctions::MapFunctions();
	m_jsonfileName = "P2";
	NodeSelector = NodeSelector::NodeSelector(map);
}

// Called when the game starts or when spawned
void AKinematicPoint::BeginPlay()
{

	Super::BeginPlay();

	// Create the map 
	//Obstacle obs = Obstacle::Obstacle();
	//map = MapFunctions::MapFunctions();
	map.ParseJson("P2");
	DrawObstacles(map.obstacles, map);
	DrawMap(map.bounding_box, map);

	// Change to link to da graph and get all initial positiona and speed values from there
	//if (!m_jsonfileName.IsEmpty())
	//	ParseJson(m_jsonfileName);

}

void AKinematicPoint::DrawGraph() {
	SetActorLocation(FVector(1.0f, 2.0f, GetActorLocation().Z), false);
	FVector location = GetActorLocation();
	location.Z = 0;
	float x = 10;
	float y = 15;
	UE_LOG(LogTemp, Display, TEXT("started rrt"));
	//NodeSelector.RandomPosition(x, y);

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
			if (NodeSelector.nodes[i]->point.Z == 0.0) {
				UE_LOG(LogTemp, Display, TEXT("FOUND 0 IN current"));
				continue;
			}
			if (parent->point.Z == 0.0) {
				UE_LOG(LogTemp, Display, TEXT("FOUND 0 IN parent"));
				continue;
			}
			DrawDebugLine(world, NodeSelector.nodes[i]->point, parent->point, FColor::Red, true);
		}
		DrawDebugSphere(world, NodeSelector.nodes[NodeSelector.nodes.Num() - 1]->point, 5.f, 26, FColor::Blue, true);
	}
	//Move that shit
	NodeSelector.GetRrtPath(path);
	HasGoalPosition = true;
	DrawDebugLines();

}

// Called every frame
void AKinematicPoint::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (HasGoalPosition) {
		//Calculate path
		MoveToPosition(path[0]->point.X, path[0]->point.Y);
		FVector XYLoc = GetActorLocation();
		XYLoc.Z = 0.f;
		//path[0].Z = 0.f;
		//UE_LOG(LogTemp, Display, TEXT("Distance To Goal: %f"), *(XYLoc - GoalPosition).SizeSquared());
		//Check if GoalPosition is reached
		if (NodeSelector.PointDistance(XYLoc, path[0]->point) <= 0.1f) {
			// goal reached, check if path is not empty
			//UE_LOG(LogTemp, Display, TEXT("Reached node %f, %f"), XYLoc.X, XYLoc.Y);
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


void AKinematicPoint::DrawObstacles(std::vector<Obstacle> obs, MapFunctions map) {
	const UWorld *world = GetWorld();
	for (int i = 0; i < obs.size(); ++i){
		Obstacle obstocheck = obs[i];
		for (int j = 0; j < obstocheck.points.size(); ++j) {
			// Draw from the last to the first 
			if(j==(obstocheck.points.size()-1)){
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], map.z), FVector(obstocheck.points[0][2], obstocheck.points[0][3], map.z), FColor::Emerald, true);
			}
			//Otherwise we always draw to the next one
			else {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], map.z), FVector(obstocheck.points[j+1][2], obstocheck.points[j+1][3], map.z), FColor::Emerald, true);
			}
		}
	}
}

void AKinematicPoint::DrawMap(Obstacle obs, MapFunctions map) {
	const UWorld *world = GetWorld();
	for (int j = 0; j < obs.points.size(); ++j) {
		// Draw from the last to the first 
		if (j == (obs.points.size() - 1)) {
			DrawDebugLine(world, FVector(obs.points[j][2], obs.points[j][3], map.z), FVector(obs.points[0][2], obs.points[0][3], map.z), FColor::Emerald, true);
		}
		//Otherwise we always draw to the next one
		else {
			DrawDebugLine(world, FVector(obs.points[j][2], obs.points[j][3], map.z), FVector(obs.points[j + 1][2], obs.points[j + 1][3], map.z), FColor::Emerald, true);
		}
	}
}

void AKinematicPoint::DrawDebugLines() {
	if (path.Num() != 0) {
		const UWorld *world = GetWorld();
		FVector current;
		FVector next;
		DrawDebugLine(world, GetActorLocation(), FVector(path[0]->point.X, path[0]->point.Y, GetActorLocation().Z), FColor::Emerald, true);
		for (int i = 0; i < path.Num() - 1; ++i) {
			current = FVector(path[i]->point.X, path[i]->point.Y, GetActorLocation().Z);
			next = FVector(path[i + 1]->point.X, path[i + 1]->point.Y, GetActorLocation().Z);
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
	path.Add(new Node(FVector(x, y, 0.f)));
	HasGoalPosition = true;
	DrawDebugLines();
}

void AKinematicPoint::GetPath() {
	//NodeSelector.GetPath(path);
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
