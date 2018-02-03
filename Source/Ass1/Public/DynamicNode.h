// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class ASS1_API DynamicNode
{
public:
	DynamicNode* parent;
	FVector point;
	FVector Velocity;
	FVector Acceleration;

	DynamicNode();
	DynamicNode(FVector);
	DynamicNode(const DynamicNode&); //copy constructor
	DynamicNode(DynamicNode, FVector);
	DynamicNode(DynamicNode*, FVector);
	~DynamicNode();
	DynamicNode(DynamicNode parent, FVector point, FVector velocity, FVector acceleration);
	DynamicNode(DynamicNode parent, FVector point, FVector velocity);
	
};
