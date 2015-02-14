// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <moai-bullet/host.h>
#include <moai-bullet/headers.h>

//================================================================//
// aku-bullet
//================================================================//

//----------------------------------------------------------------//
void MOAIBulletAppFinalize () {
};

//----------------------------------------------------------------//
void MOAIBulletAppInitialize () {	

};
//----------------------------------------------------------------//
void MOAIBulletContextInitialize () {
	REGISTER_LUA_CLASS ( MOAIBulletWorld )
	REGISTER_LUA_CLASS ( MOAIBulletShape )
	REGISTER_LUA_CLASS ( MOAIBulletTransform )
	//REGISTER_LUA_CLASS ( MOAIBulletConstraint )
};
//----------------------------------------------------------------//
