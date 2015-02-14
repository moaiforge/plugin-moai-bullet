// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"

#include <moai-bullet/MOAIBulletTransform.h>
#include <bullet/src/LinearMath/BtIdebugDraw.h>
#include <bullet/src/btBulletDynamicsCommon.h>

//----------------------------------------------------------------//
int MOAIBulletTransform::_getScale ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "U" )	

	float numb = state.GetValue < float >( 2, 0.0f );
	lua_pushnumber ( state, numb ); 
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletTransform::_setIdentity ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "U" )	
	self->mTransform->setIdentity();
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletTransform::_setEulerZYX ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "UNNN" )	

	float loc_x = state.GetValue < float >( 2, 0.0f );
	float loc_y = state.GetValue < float >( 3, 0.0f );
	float loc_z = state.GetValue < float >( 4, 0.0f );	

	self->mTransform->getBasis().setEulerZYX(btScalar(loc_x),btScalar(loc_y),btScalar(loc_z));
	
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletTransform::_setOrigin ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "UNNN" )	

	float loc_x = state.GetValue < float >( 2, 0.0f );
	float loc_y = state.GetValue < float >( 3, 0.0f );
	float loc_z = state.GetValue < float >( 4, 0.0f );	

	self->mTransform->setOrigin(btVector3((loc_x), (loc_y), (loc_z)));
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletTransform::_doOffset ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "UNNN" )	
	float off_x = state.GetValue < float >( 2, 0.0f );
	float off_y = state.GetValue < float >( 3, 0.0f );
	float off_z = state.GetValue < float >( 4, 0.0f );
	btVector3 in	= btVector3 ( off_x,off_y,off_z);
	btVector3 out   = self->mTransform->getOrigin();	
	self->mTransform->setOrigin(btVector3(btScalar(out.x()+in.x()),btScalar(out.y()+in.y()),btScalar(in.z()+out.z())));
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletTransform::_destroy ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletTransform, "U" )	
	return 0;
};
//----------------------------------------------------------------//
void MOAIBulletTransform::Destroy () {
	if ( this->mTransform ) {	
	}
}
//----------------------------------------------------------------//
MOAIBulletTransform::MOAIBulletTransform () :
	mTransform ( 0 ) {	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAILuaObject )
	RTTI_END
	this->mTransform = new btTransform;
	mTransform->setIdentity();

}
//----------------------------------------------------------------//
MOAIBulletTransform::~MOAIBulletTransform () {

	//THIS IS WRONG?
	if ( this->mTransform ) {	
		//this->LuaRelease ( this->mTransform  );
	}	
	delete (this->mTransform);
	this->Destroy ();
}
//----------------------------------------------------------------//
void MOAIBulletTransform::RegisterLuaClass ( MOAILuaState& state ) {
}
//----------------------------------------------------------------//
void MOAIBulletTransform::RegisterLuaFuncs ( MOAILuaState& state ) {
	luaL_Reg regTable [] = {
		{ "destroy",					_destroy },
		{ "setIdentity",				_setIdentity },
		{ "setOrigin",					_setOrigin },
		{ "setEulerZYX",				_setEulerZYX },
		{ "getScale",					_getScale },	
		{ "doOffset",					_doOffset },		
		{ NULL, NULL }
	};	
	luaL_register ( state, 0, regTable );
}

