// Fill out your copyright notice in the Description page of Project Settings.

#include "CarNode.h"

CarNode::CarNode()
{
}

CarNode::CarNode(CarNode* parent, FVector location) {
	this->parent = parent;
	this->point = location;
}

CarNode::CarNode(const CarNode& copied) {
	this->parent = copied.parent;
	this->point = copied.point;
	this->orientation = copied.orientation;
}

CarNode::CarNode(const CarNode& copied, FVector location, FVector orientation) {
	this->parent = copied.parent;
	this->point = location;
	this->orientation = orientation;
}

CarNode::CarNode(CarNode CarNode, FVector point) {
	this->parent = &CarNode;
	this->point = point;
}

CarNode::CarNode(FVector location) {
	this->point = location;
}

CarNode::CarNode(FVector location, FVector orientation) {
	this->point = location;
	this->orientation = orientation;
}

CarNode::CarNode(CarNode* parent, FVector location, FVector orientation) {
	this->parent = parent;
	this->point = location;
	this->orientation = orientation;
}


CarNode::~CarNode()
{
}
