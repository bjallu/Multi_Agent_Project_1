// Fill out your copyright notice in the Description page of Project Settings.

#include "KinematicCar.h"
#include "UnrealEngine.h"
#include "Components/SphereComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ConstructorHelpers.h"
#include "GameFramework/Pawn.h"
#include "Classes/Components/StaticMeshComponent.h"


// Sets default values
AKinematicCar::AKinematicCar()
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

	MovementComponent = CreateDefaultSubobject<UKinematicPointMovementComponent>(TEXT("CustomMovementComponent"));
	MovementComponent->UpdatedComponent = RootComponent;

	map = MapFunctions::MapFunctions();
	NodeSelector = NodeSelector::NodeSelector(map);
}

// Called when the game starts or when spawned
void AKinematicCar::BeginPlay()
{
	Super::BeginPlay();

}

float AKinematicCar::CalculateDistanceToGoal(const FVector XYLoc) {
	float distance = sqrt(pow(XYLoc.X - path[0]->point.X, 2) + pow(XYLoc.Y - path[0]->point.Y, 2));
	for (int i = 0; i < path.Num() - 1; ++i) {
		distance += sqrt(pow(path[i]->point.X - path[i + 1]->point.X, 2) + pow(path[i]->point.Y - path[i + 1]->point.Y, 2));
	}
	return distance;
}

// Called every frame
void AKinematicCar::Tick(float DeltaTime)
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

void AKinematicCar::DrawDebugLines() {
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
void AKinematicCar::SetupPlayerInputComponent(UInputComponent* InputComponent)
{
	//Super::SetupPlayerInputComponent(InputComponent);

	InputComponent->BindAxis("MoveForward", this, &AKinematicCar::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &AKinematicCar::MoveRight);
	//InputComponent->BindAxis("Turn", this, &AKinematicCar::Turn);
	InputComponent->BindAction("RandomDirection", IE_Pressed, this, &AKinematicCar::RandomTurn);
	InputComponent->BindAction("RandomPosition", IE_Pressed, this, &AKinematicCar::RandomPosition);
	InputComponent->BindAction("RandomPath", IE_Pressed, this, &AKinematicCar::GetPath);
	InputComponent->BindAction("DrawGraph", IE_Pressed, this, &AKinematicCar::DrawGraph);

}

void AKinematicCar::DrawGraph() {
	SetActorLocation(FVector(1.0f, 2.0f, GetActorLocation().Z), false);
	FVector location = GetActorLocation();
	location.Z = 0;
	float x = 10;
	float y = 15;							// Change to read from json
	//NodeSelector.RandomPosition(x, y);
	FVector goal = FVector(x, y, GetActorLocation().Z);
	NodeSelector.carRrt(goal, location, FVector(0.5,-0.5, GetActorLocation().Z), FVector(0.9,-0.2, GetActorLocation().Z));

	//UE_LOG(LogTemp, Display, TEXT("%f,%f"), x, y)
		//NodeSelector.differentialRrt(goal, location, PI/2, 0.0);
	const UWorld * world = GetWorld();
	//UE_LOG(LogTemp, Display, TEXT("GREEN NODE: %f,%f"), x, y);
	//DrawDebugSphere(world, FVector(x, y, GetActorLocation().Z), 2, 26, FColor::Green, true);

	if (NodeSelector.CarNodes.Num() != 0) {
		FVector current;
		FVector next;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NodeSelector.CarNodes[0]->point.X, NodeSelector.CarNodes[0]->point.Y);

		for (int i = 1; i < NodeSelector.CarNodes.Num(); ++i) {
			CarNode* parent = NodeSelector.CarNodes[i]->parent;
			//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent->point.X, parent->point.Y);
			NodeSelector.CarNodes[i]->point.Z = GetActorLocation().Z+1;
			
			parent->point.Z = GetActorLocation().Z + 1;
			//continue;
			
			//UE_LOG(LogTemp, Display, TEXT("HEIGHT %f"), NodeSelector.CarNodes[i]->point.Z);
			DrawDebugLine(world, NodeSelector.CarNodes[i]->point, parent->point, FColor::Red, true);
		}
		//DrawDebugSphere(world, NodeSelector.CarNodes[NodeSelector.CarNodes.Num() - 1]->point, 0.1f, 26, FColor::Blue, true);

	}
	goal.Z += 1;
	DrawDebugSphere(world, goal, 0.1f, 26, FColor::Green, true);
	//Move that shit

	NodeSelector.getCarRrtPath(path);
	HasGoalPosition = true;
	DrawDebugLines();


}

UPawnMovementComponent* AKinematicCar::GetMovementComponent() const
{
	return MovementComponent;
}

void AKinematicCar::MoveForward(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{

		MovementComponent->AddInputVector(GetActorForwardVector() * AxisValue);
	}

}

void AKinematicCar::MoveRight(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += AxisValue;
		SetActorRotation(NewRotation);
	}


}

void AKinematicCar::Turn(float AxisValue)
{
	FRotator NewRotation = SpringArm->RelativeRotation;
	NewRotation.Yaw += AxisValue;
	SpringArm->SetRelativeRotation(NewRotation);
}

void AKinematicCar::RandomTurn() {
	FRotator NewRotation = GetActorRotation();
	NewRotation.Yaw += rand();
	SetActorRotation(NewRotation);
}

void AKinematicCar::RandomPosition() {
	float x;
	float y;
	this->NodeSelector.RandomPosition(x, y);
	//UE_LOG(LogTemp,Display, TEXT("X: %f Y: %f"), x, y);
	//path.Add(new Node(FVector(x, y, 0.f)));
	HasGoalPosition = true;
	DrawDebugLines();
}

void AKinematicCar::GetPath() {
	//NodeSelector.GetPath(path);
	HasGoalPosition = true;
	DrawDebugLines();
}


void AKinematicCar::MoveToPosition(float x, float y) {
	//Calculate rotation
	FVector MoveTo = FVector(x, y, 0);
	FVector Forward = MoveTo - GetActorLocation();
	Forward.Z = 0;
	FRotator Rotation = FRotationMatrix::MakeFromX(Forward).Rotator();
	SetActorRotation(Rotation);

}