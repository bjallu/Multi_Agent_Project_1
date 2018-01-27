// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicPointMovementComponent.h"




void UDynamicPointMovementComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Make sure that everything is still valid, and that we are allowed to move.
	if (!PawnOwner || !UpdatedComponent || ShouldSkipUpdate(DeltaTime))
	{
		return;
	}
	// Get (and then clear) the movement vector that we set
	FVector Direction = ConsumeInputVector().GetClampedToMaxSize(1.0f);

	FVector DesiredMovement; 
	if (!Direction.IsNearlyZero())
	{
		LastPressedDirection = Direction;
		FHitResult Hit;
		// We have reached maximum velocity
		
		if (Velocity.Size() >= MaxVelocity) {
			//Calculate new position
			DesiredMovement = Direction * MaxVelocity;
			SafeMoveUpdatedComponent(DesiredMovement, UpdatedComponent->GetComponentRotation(), true, Hit);
		}
		else {
			Velocity = Velocity + (DeltaTime*Acceleration);
			DesiredMovement = Direction * Velocity.Size();
			SafeMoveUpdatedComponent(DesiredMovement, UpdatedComponent->GetComponentRotation(), true, Hit);
		}	
		UpdateComponentVelocity();
		// If we bumped into something, try to slide along it
		if (Hit.IsValidBlockingHit())
		{
			SlideAlongSurface(Direction, 1.f - Hit.Time, Hit.Normal, Hit);
		}
	}
	else {
		if (!Velocity.IsNearlyZero(DeltaTime*Acceleration)) {
			Velocity = Velocity - (DeltaTime*Acceleration);
			FHitResult Hit;
			DesiredMovement = LastPressedDirection * Velocity.Size();
			SafeMoveUpdatedComponent(DesiredMovement, UpdatedComponent->GetComponentRotation(), true, Hit);
		}
		else {
			Velocity = FVector(0.f, 0.f, 0.f);
			LastPressedDirection = FVector(0.f, 0.f, 0.f);
			
		}
		UpdateComponentVelocity();
		
	}


	
};

