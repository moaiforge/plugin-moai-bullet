// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"

#include <bullet/src/btBulletDynamicsCommon.h>

#include <moai-bullet/MOAIBulletBody.h>
#include <moai-bullet/MOAIBulletJoint.h>
#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletJointCone.h>

//----------------------------------------------------------------//
int MOAIBulletJointCone::_setLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNNN" )
	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}
	//btScalar _swingSpan1,
	//btScalar _swingSpan2,
	//btScalar _twistSpan,
	//btScalar _softness = 1.f, 
	//btScalar _biasFactor = 0.3f, 
	//btScalar _relaxationFactor = 1.0f
	float a = state.GetValue < float >( 2, 0.0f );	
	float b = state.GetValue < float >( 3, 0.0f );	
	float c = state.GetValue < float >( 4, 0.0f );	
	btConeTwistConstraint* joint = ( btConeTwistConstraint* )self->mJoint;
	joint->setLimit(btScalar(a),btScalar(b),btScalar(c));	
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletJointCone::_setLinLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNN" )
	float unitsToMeters = self->GetUnitsToMeters ();
	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}
	
	float lower = state.GetValue < float >( 2, 0.0f );	
	float upper = state.GetValue < float >( 3, 0.0f );	
	btConeTwistConstraint* joint = ( btConeTwistConstraint* )self->mJoint;
	//joint->setLowerLinLimit(lower);
	//joint->setUpperLinLimit(upper);	
	//state.Push ( 1 );
	
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletJointCone::_setAngLimit ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletJoint, "UNN" )
	float unitsToMeters = self->GetUnitsToMeters ();

	if ( !self->mJoint ) {
		MOAILog ( state, MOAILogMessages::MOAIBox2DJoint_MissingInstance );
		return 0;
	}	
	float lower = state.GetValue < float >( 2, 0.0f );	
	float upper = state.GetValue < float >( 3, 0.0f );
	btConeTwistConstraint* joint = ( btConeTwistConstraint* )self->mJoint;	
	//joint->setLowerAngLimit(-SIMD_PI / lower);
	//joint->setUpperAngLimit( SIMD_PI / upper);	
	//state.Push ( 1 );
	return 1;
}
//----------------------------------------------------------------//
MOAIBulletJointCone::MOAIBulletJointCone () {	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAIBulletJoint )
	RTTI_END
}
//----------------------------------------------------------------//
MOAIBulletJointCone::~MOAIBulletJointCone () {
}
//----------------------------------------------------------------//
void MOAIBulletJointCone::RegisterLuaClass ( MOAILuaState& state ) {
	MOAIBulletJoint::RegisterLuaClass ( state );
}
//----------------------------------------------------------------//
void MOAIBulletJointCone::RegisterLuaFuncs ( MOAILuaState& state ) {
	MOAIBulletJoint::RegisterLuaFuncs ( state );

	luaL_Reg regTable [] = {	
		{ "setLimit",						_setLimit}, 
		{ "setLinLimit",					_setLinLimit}, 
		{ "setAngLimit",					_setAngLimit}, 
		{ NULL, NULL },
	};	
	luaL_register ( state, 0, regTable );
}
