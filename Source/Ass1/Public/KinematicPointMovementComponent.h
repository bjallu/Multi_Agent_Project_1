// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PawnMovementComponent.h"
#include "KinematicPointMovementComponent.generated.h"

/**
 * 
 */
UCLASS()
class ASS1_API UKinematicPointMovementComponent : public UPawnMovementComponent
{
	GENERATED_BODY()
	
public:
	float MaxVelocity = 1.f;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
	
	
};
