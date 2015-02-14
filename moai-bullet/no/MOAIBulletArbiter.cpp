// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"


#include <bullet/src/LinearMath/BtIdebugDraw.h>
#include <moai-bullet/MOAIBulletArbiter.h>;
#include <moai-bullet/MOAIBulletBody.h>;
#include <moai-bullet/MOAIBulletWorld.h>;

//================================================================//
// local
//================================================================//

int MOAIBulletArbiter::_getContactNormal ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletArbiter, "U" )	
	//state.Push ( self->mContactNormal.x );
	//state.Push ( self->mContactNormal.y );
	return 2;
}

//----------------------------------------------------------------//

int MOAIBulletArbiter::_getNormalImpulse ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletArbiter, "U" )

	//float impulse = self->mNormalImpulse;
	//const float metersToUnits = 1 / self->GetUnitsToMeters();
	//impulse = impulse * metersToUnits;
	//state.Push ( impulse );
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletArbiter::_getTangentImpulse ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletArbiter, "U" )

	//float impulse = self->mTangentImpulse;
	//const float metersToUnits = 1 / self->GetUnitsToMeters();
	//impulse = impulse * metersToUnits;
	//state.Push ( impulse );
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletArbiter::_setContactEnabled ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletArbiter, "U" )
	
	//bool enabled = state.GetValue < bool >( 2, false );
	//self->mContact->SetEnabled ( enabled );
	
	return 0;
}

//================================================================//
// MOAIBox2DArbiter
//================================================================//

//----------------------------------------------------------------//
void MOAIBulletArbiter::BeginContact ( ) {
	
	//this->mContact = contact;
	//this->mImpulse = 0;
	//
	//this->mContactNormal = b2Vec2();
	//this->mNormalImpulse = 0.0f;
	//this->mTangentImpulse = 0.0f;
	//
	//b2Fixture* fixtureA = contact->GetFixtureA ();
	//b2Fixture* fixtureB = contact->GetFixtureB ();
	//
	//MOAIBox2DFixture* moaiFixtureA = ( MOAIBox2DFixture* )fixtureA->GetUserData ();
	//MOAIBox2DFixture* moaiFixtureB = ( MOAIBox2DFixture* )fixtureB->GetUserData ();
	//
	//moaiFixtureA->HandleCollision ( BEGIN, moaiFixtureB, this );
	//moaiFixtureB->HandleCollision ( BEGIN, moaiFixtureA, this );
}

//----------------------------------------------------------------//
void MOAIBulletArbiter::EndContact (  ) {
	
	//this->mContact = contact;
	//this->mImpulse = 0;
	//
	//b2Fixture* fixtureA = contact->GetFixtureA ();
	//b2Fixture* fixtureB = contact->GetFixtureB ();
	//
	//MOAIBox2DFixture* moaiFixtureA = ( MOAIBox2DFixture* )fixtureA->GetUserData ();
	//MOAIBox2DFixture* moaiFixtureB = ( MOAIBox2DFixture* )fixtureB->GetUserData ();
	//
	//moaiFixtureA->HandleCollision ( END, moaiFixtureB, this );
	//moaiFixtureB->HandleCollision ( END, moaiFixtureA, this );
}

//----------------------------------------------------------------//
float MOAIBulletArbiter::GetUnitsToMeters ( ) const {

	//if (this->mWorld) {
	//	return this->mWorld->GetUnitsToMeters();
	//} else {
	//	return 1.0f;
	//}

		return 1.0f;
}

//----------------------------------------------------------------//
MOAIBulletArbiter::MOAIBulletArbiter ( ) :
	mWorld ( NULL ) {
	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAILuaObject )
	RTTI_END
}

//----------------------------------------------------------------//
MOAIBulletArbiter::MOAIBulletArbiter ( const MOAIBulletWorld& world ) :
	//mContact ( 0 ),
	//mImpulse ( 0 ),
	mWorld ( &world ) {	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAILuaObject )
	RTTI_END
}

//----------------------------------------------------------------//
MOAIBulletArbiter::~MOAIBulletArbiter () {
}

//----------------------------------------------------------------//
void MOAIBulletArbiter::PostSolve ( ) {

	/*this->mContact = contact;
	this->mImpulse = impulse;

	b2Fixture* fixtureA = contact->GetFixtureA ();
	b2Fixture* fixtureB = contact->GetFixtureB ();
	
	MOAIBox2DFixture* moaiFixtureA = ( MOAIBox2DFixture* )fixtureA->GetUserData ();
	MOAIBox2DFixture* moaiFixtureB = ( MOAIBox2DFixture* )fixtureB->GetUserData ();
		
	b2WorldManifold* worldManifold = new b2WorldManifold ();
	contact->GetWorldManifold ( worldManifold );
	this->mContactNormal = worldManifold->normal;
	delete worldManifold;
	
	b2Manifold* manifold = contact->GetManifold ();
	u32 totalPoints = manifold->pointCount;
	
	this->mNormalImpulse = 0.0f;
	this->mTangentImpulse = 0.0f;
	
	for ( u32 i = 0; i < totalPoints; ++i ) {
		this->mNormalImpulse += impulse->normalImpulses [ i ];
		this->mTangentImpulse += impulse->tangentImpulses [ i ];
	}
	
	moaiFixtureA->HandleCollision ( POST_SOLVE, moaiFixtureB, this );
	moaiFixtureB->HandleCollision ( POST_SOLVE, moaiFixtureA, this );*/

}

//----------------------------------------------------------------//
void MOAIBulletArbiter::PreSolve (  ) {

	//UNUSED ( oldManifold );
	//
	//this->mContact = contact;
	//this->mImpulse = 0;
	//
	//b2Fixture* fixtureA = contact->GetFixtureA ();
	//b2Fixture* fixtureB = contact->GetFixtureB ();
	//
	//MOAIBox2DFixture* moaiFixtureA = ( MOAIBox2DFixture* )fixtureA->GetUserData ();
	//MOAIBox2DFixture* moaiFixtureB = ( MOAIBox2DFixture* )fixtureB->GetUserData ();
	//
	//moaiFixtureA->HandleCollision ( PRE_SOLVE, moaiFixtureB, this );
	//moaiFixtureB->HandleCollision ( PRE_SOLVE, moaiFixtureA, this );
}

//----------------------------------------------------------------//
void MOAIBulletArbiter::RegisterLuaClass ( MOAILuaState& state ) {

	state.SetField ( -1, "ALL", ( u32 )ALL );
	state.SetField ( -1, "BEGIN", ( u32 )BEGIN );
	state.SetField ( -1, "END", ( u32 )END );
	state.SetField ( -1, "PRE_SOLVE", ( u32 )PRE_SOLVE );
	state.SetField ( -1, "POST_SOLVE", ( u32 )POST_SOLVE );
	
	luaL_Reg regTable [] = {
		{ "new",							MOAILogMessages::_alertNewIsUnsupported },
		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAIBulletArbiter::RegisterLuaFuncs ( MOAILuaState& state ) {

	luaL_Reg regTable [] = {
		{ "getContactNormal",			_getContactNormal },
		{ "getNormalImpulse",			_getNormalImpulse },
		{ "getTangentImpulse",			_getTangentImpulse },
		{ "setContactEnabled",			_setContactEnabled },
		{ "new",						MOAILogMessages::_alertNewIsUnsupported },
		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}
