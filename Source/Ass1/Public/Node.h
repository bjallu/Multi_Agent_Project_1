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
	FVector orientation;

	Node();
	Node(FVector);
	Node(const Node&); //copy constructor
	Node(Node, FVector);
	Node(Node*, FVector);
	~Node();
	Node(Node* parent, FVector location, FVector orientation);
	Node(FVector, FVector);
	Node(const Node& copied, FVector location, FVector orientation);
};
