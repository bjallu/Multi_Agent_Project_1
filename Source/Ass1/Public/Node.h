// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class ASS1_API Node
{
public:
	Node* parent;
	FVector point;
	float orientation;

	Node();
	Node(FVector);
	Node(const Node&); //copy constructor
	Node(Node, FVector);
	Node(Node*, FVector);
	~Node();
	Node(Node* parent, FVector location, float orientation);
	Node(FVector, float);
	Node(const Node& copied, FVector location, float orientation);
};
