// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"

#include <bullet/src/btBulletDynamicsCommon.h>

#include <moai-bullet/MOAIBulletBody.h>
#include <moai-bullet/MOAIBulletJoint.h>
#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletJointHinge.h>

//----------------------------------------------------------------//
int MOAIBulletJointHinge::_setLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNN" )
	float unitsToMeters = self->GetUnitsToMeters ();
	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}
	float lower = state.GetValue < float >( 2, 0.0f );	
	float upper = state.GetValue < float >( 3, 0.0f );	
	btHingeConstraint* joint = ( btHingeConstraint* )self->mJoint;
	joint->setLimit(btScalar(lower),btScalar(upper));
	//state.Push ( 1 );	
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletJointHinge::_setLinLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNN" )
	float unitsToMeters = self->GetUnitsToMeters ();
	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}	
	float lower = state.GetValue < float >( 2, 0.0f );	
	float upper = state.GetValue < float >( 3, 0.0f );	
	btHingeConstraint* joint = ( btHingeConstraint* )self->mJoint;
	//state.Push ( 1 );
	
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletJointHinge::_setAngLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNN" )
	float unitsToMeters = self->GetUnitsToMeters ();
	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}	
	float lower = state.GetValue < float >( 2, 0.0f );	
	float upper = state.GetValue < float >( 3, 0.0f );	
	btHingeConstraint* joint = ( btHingeConstraint* )self->mJoint;	
	//state.Push ( 1 );
	return 1;
}
//----------------------------------------------------------------//
MOAIBulletJointHinge::MOAIBulletJointHinge () {	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAIBulletJoint )
	RTTI_END
}
//----------------------------------------------------------------//
MOAIBulletJointHinge::~MOAIBulletJointHinge () {
}
//----------------------------------------------------------------//
void MOAIBulletJointHinge::RegisterLuaClass ( MOAILuaState& state ) {
	MOAIBulletJoint::RegisterLuaClass ( state );
}
//----------------------------------------------------------------//
void MOAIBulletJointHinge::RegisterLuaFuncs ( MOAILuaState& state ) {
	MOAIBulletJoint::RegisterLuaFuncs ( state );

	luaL_Reg regTable [] = {		
		{ "setLimit",						_setLimit}, 
		{ "setLinLimit",					_setLinLimit}, 
		{ "setAngLimit",					_setAngLimit}, 
		{ NULL, NULL },
	};	
	luaL_register ( state, 0, regTable );
}
