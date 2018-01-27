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

	// Take control of the default player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	MovementComponent = CreateDefaultSubobject<UDynamicPointMovementComponent>(TEXT("CustomMovementComponent"));
	MovementComponent->UpdatedComponent = RootComponent;
	
}

// Called when the game starts or when spawned
void ADynamicPoint::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ADynamicPoint::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (HasGoalPosition) {
		//MoveToPosition only rotates the actor
		MoveToPosition(path[0].X, path[0].Y);
		FVector XYLoc = GetActorLocation();
		XYLoc.Z = 0.f;
		float distanceToStop = pow(MovementComponent->Velocity.Size(),2) / (2 * MovementComponent->Acceleration*DeltaTime);
		float distancetogoal = sqrt(pow(XYLoc.X - path[0].X, 2) + pow(XYLoc.Y - path[0].Y, 2));
		UE_LOG(LogTemp, Display, TEXT("Distance left: %f DistanceToStop: %f"), distancetogoal, distanceToStop);
		if (distancetogoal <= distanceToStop) {
				// goal reached, check if path is not empty
				if (distancetogoal < 0.1f) {
					path.RemoveAt(0);
					if (path.Num() == 0) {
						HasGoalPosition = false;
						FlushPersistentDebugLines(GetWorld());
					}
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
		DrawDebugLine(world, GetActorLocation(), FVector(path[0].X, path[0].Y, GetActorLocation().Z), FColor::Emerald, true);
		for (int i = 0; i < path.Num() - 1; ++i) {
			current = FVector(path[i].X, path[i].Y, GetActorLocation().Z);
			next = FVector(path[i + 1].X, path[i + 1].Y, GetActorLocation().Z);
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
	InputComponent->BindAxis("Turn", this, &ADynamicPoint::Turn);
	InputComponent->BindAction("RandomDirection",IE_Pressed, this, &ADynamicPoint::RandomTurn);
	InputComponent->BindAction("RandomPosition", IE_Pressed, this, &ADynamicPoint::RandomPosition);
	InputComponent->BindAction("RandomPath", IE_Pressed, this, &ADynamicPoint::GetPath);
	InputComponent->BindAction("DrawGraph", IE_Pressed, this, &ADynamicPoint::DrawGraph);

}

void ADynamicPoint::DrawGraph() {
	FVector location = GetActorLocation();
	location.Z = 0;
	float x = 0;
	float y = 0;
	NodeSelector.RandomPosition(x, y);
	FVector goal = FVector(x, y, 0.f);
	NodeSelector.rrt(goal, location);
	const UWorld * world = GetWorld();

	if (NodeSelector.nodes.Num() != 0) {
		FVector current;
		FVector next;
		//UE_LOG(LogTemp, Display, TEXT("%f, %f"), NodeSelector.nodes[0]->point.X, NodeSelector.nodes[0]->point.Y);
		for (int i = 1; i < NodeSelector.nodes.Num(); ++i) {
			Node* parent = NodeSelector.nodes[i]->parent;
			//UE_LOG(LogTemp, Display, TEXT("%f, %f"), parent->point.X, parent->point.Y);
			DrawDebugLine(world, NodeSelector.nodes[i]->point, parent->point,FColor::Red, true);
		}
	}
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

void ADynamicPoint::RandomPosition() {
	float x;
	float y;
	this->NodeSelector.RandomPosition(x, y);
	//UE_LOG(LogTemp,Display, TEXT("X: %f Y: %f"), x, y);
	path.Add(FVector(x, y, 0.f));
	HasGoalPosition = true;
	DrawDebugLines();
}

void ADynamicPoint::GetPath() {
	NodeSelector.GetPath(path);
	HasGoalPosition = true;
	DrawDebugLines();
}


void ADynamicPoint::MoveToPosition(float x, float y) {
	//Calculate rotation
	FVector MoveTo = FVector(x, y, 0);
	FVector Forward = MoveTo - GetActorLocation();
	Forward.Z = 0;
	FRotator Rotation = FRotationMatrix::MakeFromX(Forward).Rotator();
	SetActorRotation(Rotation);

}