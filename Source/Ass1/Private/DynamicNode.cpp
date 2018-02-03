// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicNode.h"

DynamicNode::DynamicNode()
{
}

DynamicNode::DynamicNode(DynamicNode* parent, FVector location) {
	this->parent = parent;
	this->point = location;
}

DynamicNode::DynamicNode(const DynamicNode& copied) {
	this->parent = copied.parent;
	this->point = copied.point;
}


DynamicNode::DynamicNode(DynamicNode DynamicNode, FVector point) {
	this->parent = &DynamicNode;
	this->point = point;
}

DynamicNode::DynamicNode(FVector location) {
	this->point = location;
}

DynamicNode::DynamicNode(DynamicNode parent, FVector point, FVector velocity, FVector acceleration) {
	this->parent = &parent;
	this->point = point;
	this->Velocity = velocity;
	this->Acceleration = acceleration;
}




DynamicNode::~DynamicNode()
{
}
