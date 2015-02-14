// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"

#include <bullet/src/btBulletDynamicsCommon.h>

#include <moai-bullet/MOAIBulletBody.h>
#include <moai-bullet/MOAIBulletJoint.h>
#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletWheel.h>


//*******************************************
//ENGINE INFORMATIO
//float	gEngineForce			= 00.f;
//float	gBreakingForce			= 00.f;
//float	maxEngineForce			= 1000.f;//this should be engine/velocity dependent
//float	maxBreakingForce		= 100.f;
//float	gVehicleSteering		= 00.f;
//float	steeringIncrement		= 0.01f;
//float	steeringClamp			= 0.3f;



///*******************************************
//APPLY TO ALL TIRES FROM TUNNER

	//float	wheelFriction			= 1000;//BT_LARGE_FLOAT;
	//float	suspensionStiffness		= 20.f;
	//float	suspensionDamping		= 2.3f;
	//float	suspensionCompression	= 4.4f;
	//float	rollInfluence			= 0.1f;//1.0f;



//----------------------------------------------------------------//
int MOAIBulletWheel::_addWheelToVehicle ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "U" )
btRaycastVehicle::btVehicleTuning m_tuning; //WTF
self->mVehicle->addWheel(self->mConnect,self->mWheelDirection,self->mWheelAxle,self->mSuspensionRestLength,self->mWheelRadius,m_tuning,self->mIsFrontWheel);
return 0;
};
//----------------------------------------------------------------//
int MOAIBulletWheel::_setWheelDirection ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UNNN" )
float vecA		= state.GetValue < float >( 2, 0 );
float vecB		= state.GetValue < float >( 3, 0 );
float vecC		= state.GetValue < float >( 4, 0 );		
self->mConnect = btVector3(vecA,vecB,vecC);
return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWheel::_setWheelAxle ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UNNN" )
float vecA		= state.GetValue < float >( 2, 0 );
float vecB		= state.GetValue < float >( 3, 0 );
float vecC		= state.GetValue < float >( 4, 0 );		
self->mConnect = btVector3(vecA,vecB,vecC);
return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWheel::_setWheelRadius( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UN" )
float radius		= state.GetValue < float >( 2, 0 );
self->mWheelRadius = radius;
return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWheel::_setFrontWheel( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UB" )
bool isFrontWheel	= state.GetValue < bool >( 2, 0 );
self->mIsFrontWheel = isFrontWheel;
return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWheel::_suspensionRestLength( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UN" )
float suspensionRestLength	= state.GetValue < float >( 2, 0.0f );
self->mSuspensionRestLength = suspensionRestLength;
return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWheel::_setConnectionPoint ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletWheel, "UNNN" )
float vecA		= state.GetValue < float >( 2, 0 );
float vecB		= state.GetValue < float >( 3, 0 );
float vecC		= state.GetValue < float >( 4, 0 );		
self->mConnect = btVector3(vecA,vecB,vecC);
return 0;	
}
//----------------------------------------------------------------//
MOAIBulletWheel::MOAIBulletWheel () :
mVehicle(0),
mConnect(0,0,0),
mWheelDirection(0,-1,0),
mWheelAxle(-1,0,0),
mIsFrontWheel(false),
mWheelRadius(0.7f),
mSuspensionRestLength(0.6f)
{	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAIBulletVehicle )
	RTTI_END
}
//----------------------------------------------------------------//
MOAIBulletWheel::~MOAIBulletWheel () {


}
//----------------------------------------------------------------//
void MOAIBulletWheel::RegisterLuaClass ( MOAILuaState& state ) {
	


}
//----------------------------------------------------------------//
void MOAIBulletWheel::RegisterLuaFuncs ( MOAILuaState& state ) {
	
	luaL_Reg regTable [] = {		
		{ "addWheelToVehicle",			_addWheelToVehicle}, 

		{ "setConnectionPoint",			_setConnectionPoint}, 

		{ "setWheelDirection",			_setWheelDirection}, 
		{ "setWheelAxle",				_setWheelAxle}, 
		{ "setWheelRadius",				_setWheelRadius},
		{ "setFrontWheel",				_setFrontWheel},
		{ "suspensionRestLength",		_suspensionRestLength},		

		{ NULL, NULL },
	};	
	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAIBulletWheel::setTune (btDiscreteDynamicsWorld* world) {
	//this->mTune = world;
};


void MOAIBulletWheel::setVehicle (btRaycastVehicle *vehicle) {
	this->mVehicle = vehicle;
};


	static int		_setConnectionPoint			( lua_State* L );
	static int		_setWheelDirection			( lua_State* L );
	static int		_wheelAxle					( lua_State* L );
	static int		_setWheelAxle				( lua_State* L );
	static int		_setWheelRadius				( lua_State* L );
	static int		_setFrontWheel				( lua_State* L );
	static int		_suspensionRestLength		( lua_State* L );
