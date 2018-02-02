// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NodeSelector.h"
#include "GameFramework/Pawn.h"
#include "DynamicPointMovementComponent.h"
#include "DrawDebugHelpers.h"
#include "Node.h"
#include "DifferentialDrive.generated.h"
/**/
UCLASS()
class ASS1_API ADifferentialDrive : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	class UDynamicPointMovementComponent* MovementComponent;
	class USpringArmComponent* SpringArm;
	bool HasGoalPosition = false;
	FVector GoalPosition;
	NodeSelector NodeSelector;
	TArray<Node*> path;
	ADifferentialDrive();

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
	float CalculateDistanceToGoal(const FVector);
	
};
