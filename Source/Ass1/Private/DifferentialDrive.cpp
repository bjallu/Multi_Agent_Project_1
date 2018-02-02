// Fill out your copyright notice in the Description page of Project Settings.

#include "DifferentialDrive.h"
#include "UnrealEngine.h"
#include "Components/SphereComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ConstructorHelpers.h"
#include "GameFramework/Pawn.h"
#include "Classes/Components/StaticMeshComponent.h"


// Sets default values
ADifferentialDrive::ADifferentialDrive()
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

}

// Called when the game starts or when spawned
void ADifferentialDrive::BeginPlay()
{
	Super::BeginPlay();

}

float ADifferentialDrive::CalculateDistanceToGoal(const FVector XYLoc) {
	float distance = sqrt(pow(XYLoc.X - path[0]->point.X, 2) + pow(XYLoc.Y - path[0]->point.Y, 2));
	for (int i = 0; i < path.Num() - 1; ++i) {
		distance += sqrt(pow(path[i]->point.X - path[i + 1]->point.X, 2) + pow(path[i]->point.Y - path[i + 1]->point.Y, 2));
	}
	return distance;
}

// Called every frame
void ADifferentialDrive::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (HasGoalPosition) {
	}
}

void ADifferentialDrive::DrawDebugLines() {
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
void ADifferentialDrive::SetupPlayerInputComponent(UInputComponent* InputComponent)
{
	//Super::SetupPlayerInputComponent(InputComponent);

	InputComponent->BindAxis("MoveForward", this, &ADifferentialDrive::MoveForward);
	InputComponent->BindAxis("MoveRight", this, &ADifferentialDrive::MoveRight);
	//InputComponent->BindAxis("Turn", this, &ADifferentialDrive::Turn);
	InputComponent->BindAction("RandomDirection", IE_Pressed, this, &ADifferentialDrive::RandomTurn);
	InputComponent->BindAction("RandomPosition", IE_Pressed, this, &ADifferentialDrive::RandomPosition);
	InputComponent->BindAction("RandomPath", IE_Pressed, this, &ADifferentialDrive::GetPath);
	InputComponent->BindAction("DrawGraph", IE_Pressed, this, &ADifferentialDrive::DrawGraph);

}

void ADifferentialDrive::DrawGraph() {
	SetActorLocation(FVector(1.0f, 2.0f, GetActorLocation().Z), false);
	FVector location = GetActorLocation();
	location.Z = 0;
	float x = 10;
	float y = 15;							// Change to read from json
	//NodeSelector.RandomPosition(x, y);
	FVector goal = FVector(x, y, 70.f);
	NodeSelector.differentialRrt(goal, location, FVector(0.5,-0.5,0.0), FVector(0.9,-0.2,0.0));

	//UE_LOG(LogTemp, Display, TEXT("%f,%f"), x, y)
		//NodeSelector.differentialRrt(goal, location, PI/2, 0.0);
	const UWorld * world = GetWorld();
	//UE_LOG(LogTemp, Display, TEXT("GREEN NODE: %f,%f"), x, y);
	DrawDebugSphere(world, FVector(x, y, GetActorLocation().Z), 2, 26, FColor::Green, true);

	if (NodeSelector.nodes.Num() != 0) {
		FVector current;
		FVector next;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NodeSelector.nodes[0]->point.X, NodeSelector.nodes[0]->point.Y);

		for (int i = 1; i < NodeSelector.nodes.Num(); ++i) {
			Node* parent = NodeSelector.nodes[i]->parent;
			//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent->point.X, parent->point.Y);
			if (NodeSelector.nodes[i]->point.Z == 0.0) {
				//UE_LOG(LogTemp, Display, TEXT("FOUND 0 IN current"));
				continue;
			}
			if (parent->point.Z == 0.0) {
				//UE_LOG(LogTemp, Display, TEXT("FOUND 0 IN parent"));
				continue;
			}
			DrawDebugLine(world, NodeSelector.nodes[i]->point, parent->point, FColor::Red, true);
		}
		DrawDebugSphere(world, NodeSelector.nodes[NodeSelector.nodes.Num() - 1]->point, 0.1f, 26, FColor::Blue, true);

	}
	//Move that shit

	NodeSelector.GetRrtPath(path);
	HasGoalPosition = true;
	DrawDebugLines();


}

UPawnMovementComponent* ADifferentialDrive::GetMovementComponent() const
{
	return MovementComponent;
}

void ADifferentialDrive::MoveForward(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{

		MovementComponent->AddInputVector(GetActorForwardVector() * AxisValue);
	}

}

void ADifferentialDrive::MoveRight(float AxisValue)
{

	if (MovementComponent && (MovementComponent->UpdatedComponent == RootComponent))
	{
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += AxisValue;
		SetActorRotation(NewRotation);
	}


}

void ADifferentialDrive::Turn(float AxisValue)
{
	FRotator NewRotation = SpringArm->RelativeRotation;
	NewRotation.Yaw += AxisValue;
	SpringArm->SetRelativeRotation(NewRotation);
}

void ADifferentialDrive::RandomTurn() {
	FRotator NewRotation = GetActorRotation();
	NewRotation.Yaw += rand();
	SetActorRotation(NewRotation);
}

void ADifferentialDrive::RandomPosition() {
	float x;
	float y;
	this->NodeSelector.RandomPosition(x, y);
	//UE_LOG(LogTemp,Display, TEXT("X: %f Y: %f"), x, y);
	path.Add(new Node(FVector(x, y, 0.f)));
	HasGoalPosition = true;
	DrawDebugLines();
}

void ADifferentialDrive::GetPath() {
	NodeSelector.GetPath(path);
	HasGoalPosition = true;
	DrawDebugLines();
}


void ADifferentialDrive::MoveToPosition(float x, float y) {
	//Calculate rotation
	FVector MoveTo = FVector(x, y, 0);
	FVector Forward = MoveTo - GetActorLocation();
	Forward.Z = 0;
	FRotator Rotation = FRotationMatrix::MakeFromX(Forward).Rotator();
	SetActorRotation(Rotation);

}