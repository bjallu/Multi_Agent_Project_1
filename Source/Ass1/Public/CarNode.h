// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class ASS1_API CarNode
{
public:
	CarNode* parent;
	FVector point;
	FVector orientation;

	CarNode();
	CarNode(FVector);
	CarNode(const CarNode&); //copy constructor
	CarNode(CarNode, FVector);
	CarNode(CarNode*, FVector);
	~CarNode();
	CarNode(CarNode* parent, FVector location, FVector orientation);
	CarNode(FVector, FVector);
	CarNode(const CarNode& copied, FVector location, FVector orientation);


};
