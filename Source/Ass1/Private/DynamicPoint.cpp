// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicPoint.h"
#include "UnrealEngine.h"
#include "Components/SphereComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ConstructorHelpers.h"
#include "GameFramework/Pawn.h"
#include "Classes/Components/StaticMeshComponent.h"


// Sets default values
ADynamicPoint::ADynamicPoint()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(1.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	//SphereComponent->SetSimulatePhysics(true);

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Cube.Shape_Cube"));
	if (SphereVisualAsset.Succeeded())
	{
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -40.f));
		SphereVisual->SetWorldScale3D(FVector(1.f));
	}
	/*
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraAttachmentArm"));
	SpringArm->SetupAttachment(RootComponent);
	SpringArm->RelativeRotation = FRotator(-45.f, 0.f, 0.f);
	SpringArm->TargetArmLength = 400.f;
	SpringArm->bEnableCameraLag = true;
	SpringArm->bInheritYaw = 0;
	SpringArm->CameraLagSpeed = 3.0f;

	// Create a camera and attach to our spring arm
	UCameraComponent* Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("ActualCamera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
	*/
	// Take control of the default player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	MovementComponent = CreateDefaultSubobject<UDynamicPointMovementComponent>(TEXT("CustomMovementComponent"));
	MovementComponent->UpdatedComponent = RootComponent;

	map = MapFunctions::MapFunctions();
//	NodeSelector = NodeSelector::NodeSelector(map);
}

// Called when the game starts or when spawned
void ADynamicPoint::BeginPlay()
{
	Super::BeginPlay();
	map.ParseJson("P1");
	DrawObstacles(map.obstacles, map);
	DrawMap(map.bounding_box, map);
}


void ADynamicPoint::DrawObstacles(std::vector<Obstacle> obs, MapFunctions map) {
	const UWorld *world = GetWorld();
	for (int i = 0; i < obs.size(); ++i) {
		Obstacle obstocheck = obs[i];
		for (int j = 0; j < obstocheck.points.size(); ++j) {
			// Draw from the last to the first 
			if (j == (obstocheck.points.size() - 1)) {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], map.z), FVector(obstocheck.points[0][2], obstocheck.points[0][3], map.z), FColor::Emerald, true);
			}
			//Otherwise we always draw to the next one
			else {
				DrawDebugLine(world, FVector(obstocheck.points[j][2], obstocheck.points[j][3], map.z), FVector(obstocheck.points[j + 1][2], obstocheck.points[j + 1][3], map.z), FColor::Emerald, true);
			}
		}
	}
}

void ADynamicPoint::DrawMap(Obstacle obs, MapFunctions map) {
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

float ADynamicPoint::CalculateDistanceToGoal(const FVector XYLoc) {
	float distance = sqrt(pow(XYLoc.X - path[0]->point.X, 2) + pow(XYLoc.Y - path[0]->point.Y, 2));
	for (int i = 0; i < path.Num()-1; ++i) {
		distance += sqrt(pow(path[i]->point.X - path[i+1]->point.X, 2) + pow(path[i]->point.Y - path[i+1]->point.Y, 2));
	}
	return distance;
}

// Called every frame
void ADynamicPoint::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (HasGoalPosition) {
		//MoveToPosition only rotates the actor
		MoveToPosition(path[0]->point.X, path[0]->point.Y);
		FVector XYLoc = GetActorLocation();
		XYLoc.Z = 0.f;
		path[0]->point.Z = 0.f;
		float distanceToStop = pow(MovementComponent->Velocity.Size(),2) / (2 * MovementComponent->Acceleration*DeltaTime);
		float distancetogoal = CalculateDistanceToGoal(XYLoc);
		float distanceToNext = sqrt(pow(XYLoc.X - path[0]->point.X, 2) + pow(XYLoc.Y - path[0]->point.Y, 2));
		//UE_LOG(LogTemp, Display, TEXT("Distance left: %f DistanceToStop: %f"), distancetogoal, distanceToStop);
		//Time to start slowing down
		if (distancetogoal <= distanceToStop) {

		}
		// goal reached, check if path is not empty
		else if (distanceToNext < 1.f) {
			path.RemoveAt(0);
			if (path.Num() == 0) {
				HasGoalPosition = false;
				FlushPersistentDebugLines(GetWorld());
			}
		}	
		
		
		//Its not time to stop, keep moving forward
		else
			MovementComponent->AddInputVector(GetActorForwardVector());
	}
}

void ADynamicPoint::DrawDebugLines() {
	if (path.Num() != 0) {
		const UWorld *world = GetWorld();
		FVector current;
		FVector next;
		DrawDebugLine(world, GetActorLocation(), FVector(path[0]->point.X, path[0]->point.Y, GetActorLocation().Z), FColor::Emerald, true);
		for (int i = 0; i < path.Num() - 1; ++i) {
			current = FVector(path[i]->point.X, path[i]->point.Y, GetActorLocation().Z+1);
			next = FVector(path[i + 1]->point.X, path[i + 1]->point.Y, GetActorLocation().Z+1);
			DrawDebugLine(world, current, next, FColor::Emerald, true);
		}
	}
}

// Called to bind functionality to input
void ADynamicPoint::SetupPlayerInputComponent(UInputComponent* InputComponent)
{
	//Super::SetupPlayerInputComponent(InputComponent);

	InputComponent->BindAxis("MoveForward", this, &ADynamicPoint::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &ADynamicPoint::MoveRight);
	//InputComponent->BindAxis("Turn", this, &ADynamicPoint::Turn);
	InputComponent->BindAction("RandomDirection",IE_Pressed, this, &ADynamicPoint::RandomTurn);
	InputComponent->BindAction("DrawGraph", IE_Pressed, this, &ADynamicPoint::DrawGraph);

}

void ADynamicPoint::DrawGraph() {
	//SetActorLocation(FVector(1.0f, 2.0f, GetActorLocation().Z), false);
	SetActorLocation(FVector(map.pos_start.X, map.pos_start.Y, map.z));
	FVector startVelocity = FVector(0.5, -0.5, 0.0);
	FVector goalVelocity = FVector(0.9, -0.2, 0.0);
	const UWorld * world = GetWorld();
	//location.Z = 0;
	float x = 10;
	float y = 15;							// Change to read from json
	FVector goal = FVector(x, y, 0.f);
	map.vel_start.Z = 0.f;
	map.vel_goal.Z = 0.f;
	NodeSelector.dynamicPointRrt(map.pos_goal, map.pos_start, map.vel_start, map.vel_goal, map, world);
	//UE_LOG(LogTemp,Display,TEXT("%f,%f"),GetActorLocation().X,GetActorLocation().Y)
	//NodeSelector.differentialRrt(goal, location, PI/2, 0.0);
	
	for (int i = 1; i < NodeSelector.DynamicNodes.Num(); ++i) {
		DynamicNode* parent = NodeSelector.DynamicNodes[i]->parent;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent->point.X, parent->point.Y);
		NodeSelector.DynamicNodes[i]->point.Z = map.z;

		parent->point.Z = map.z;
		//continue;

		//UE_LOG(LogTemp, Display, TEXT("HEIGHT %f"), NodeSelector.nodes[i]->point.Z);
		DrawDebugLine(world, NodeSelector.DynamicNodes[i]->point, parent->point, FColor::Red, true);
	}
	//Move that shit
	
	NodeSelector.GetDynamicRrtPath(path);
	HasGoalPosition = true;					// CRASHES
	//DrawDebugLines();
	
	
	
}

UPawnMovementComponent* ADynamicPoint::GetMovementComponent() const
{
	return MovementComponent;
}

void ADynamicPoint::MoveForward(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{

		MovementComponent->AddInputVector(GetActorForwardVector() * AxisValue );
	}

}

void ADynamicPoint::MoveRight(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += AxisValue;
		SetActorRotation(NewRotation);
	}

	
}

void ADynamicPoint::Turn(float AxisValue)
{
	FRotator NewRotation = SpringArm->RelativeRotation;
	NewRotation.Yaw += AxisValue;
	SpringArm->SetRelativeRotation(NewRotation);
}

void ADynamicPoint::RandomTurn() {
	FRotator NewRotation = GetActorRotation();
	NewRotation.Yaw += rand();
	SetActorRotation(NewRotation);
}






void ADynamicPoint::MoveToPosition(float x, float y) {
	//Calculate rotation
	FVector MoveTo = FVector(x, y, 0);
	FVector Forward = MoveTo - GetActorLocation();
	Forward.Z = 0;
	FRotator Rotation = FRotationMatrix::MakeFromX(Forward).Rotator();
	SetActorRotation(Rotation);

}