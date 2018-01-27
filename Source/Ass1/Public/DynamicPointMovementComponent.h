// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PawnMovementComponent.h"
#include "DynamicPointMovementComponent.generated.h"

/**
 * 
 */
UCLASS()
class ASS1_API UDynamicPointMovementComponent : public UPawnMovementComponent
{
	GENERATED_BODY()
	
public:
	float MaxVelocity = 1.f;
	float Acceleration = 1.f;
	float CurrentVelocity = 0.f;
	FVector LastPressedDirection;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
	
	
};
