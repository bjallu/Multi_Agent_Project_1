// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NodeSelector.h"
#include "GameFramework/Pawn.h"
#include "KinematicPointMovementComponent.h"
#include "DrawDebugHelpers.h"
#include "Obstacle.h"
#include <vector>
#include "KinematicPoint.generated.h"

/**
 * 
 */
UCLASS()
class ASS1_API AKinematicPoint : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	class UKinematicPointMovementComponent* MovementComponent;
	class USpringArmComponent* SpringArm;
	bool HasGoalPosition = false;
	FVector GoalPosition;
	NodeSelector NodeSelector;
	TArray<Node*> path;
	FString m_jsonfileName;
	AKinematicPoint();
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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override;

	virtual UPawnMovementComponent* GetMovementComponent() const override;

	virtual void MoveForward(float AxisValue);
	virtual void MoveRight(float AxisValue);
	virtual void Turn(float AxisValue);
	virtual void RandomTurn();
	virtual void RandomPosition();
	virtual void MoveToPosition(float x, float y);
	virtual void GetPath();
	virtual void DrawDebugLines();
	virtual void DrawGraph();
	virtual void DrawObstacles(std::vector<Obstacle>);
	virtual void DrawMap(Obstacle obs);
};
