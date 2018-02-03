// Fill out your copyright notice in the Description page of Project Settings.

#include "Node.h"

Node::Node()
{
}

Node::Node(Node* parent, FVector location) {
	this->parent = parent;
	this->point = location;
}

Node::Node(const Node& copied) {
	this->parent = copied.parent;
	this->point = copied.point;
}

Node::Node(const Node& copied, FVector location, FVector orientation) {
	this->parent = copied.parent;
	this->point = location;
	this->orientation = orientation;
}

Node::Node(Node node, FVector point) {
	this->parent = &node;
	this->point = point;
}

Node::Node(FVector location) {
	this->point = location;
}

Node::Node(FVector location, FVector orientation) {
	this->point = location;
	this->orientation = orientation;
}

Node::Node(Node* parent, FVector location, FVector orientation) {
	this->parent = parent;
	this->point = location;
	this->orientation = orientation;
}


Node::~Node()
{
}
