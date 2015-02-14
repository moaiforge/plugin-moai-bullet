// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"



#include <moai-bullet/MOAIBulletConstraint.h>
#include <moai-bullet/MOAIBulletBody.h>

#include <bullet/src/btBulletDynamicsCommon.h>


//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointHinge ( lua_State* L ) {
	//MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUNNNNNN" )	

	MOAILuaState state ( L );
	if ( !state.CheckParams ( 1, "UUNNNNNN" )) return 0;

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	btTransform localA, localB;

	localA.setIdentity(); 
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(a_x), btScalar(a_y), btScalar(a_z)));

	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(b_x), btScalar(a_y), btScalar(a_z)));
	

	btHingeConstraint*		hingeC;
	hingeC = new btHingeConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB); //NEED TO MAKE JOINT SIZE
	hingeC->setDbgDrawSize(0);

		float	targetVelocity = 1.0f;
		float	maxMotorImpulse = 1.0f;
		
		hingeC->enableAngularMotor(true,targetVelocity,maxMotorImpulse);
		hingeC->setBreakingImpulseThreshold(100.0f);
		hingeC->setLimit(-SIMD_HALF_PI * 0.5f, SIMD_HALF_PI * 0.5f);



	MOAIBulletConstraint* constraint = new MOAIBulletConstraint ();
	//constraint->mConstraint = btHingeConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB); //NEED TO MAKE JOINT SIZE
	//constraint->mConstraint->data = constraint;
	
	constraint->LuaRetain ( bodyA );
	constraint->LuaRetain ( bodyB);
	constraint->PushLuaUserdata ( state );

	//MOAICpConstraint* constraint = new MOAICpConstraint ();
	//constraint->mConstraint = cpSlideJointNew ( a->mBody, b->mBody, anchr1, anchr2, min, max );
	//constraint->mConstraint->data = constraint;
	//constraint->LuaRetain ( a );
	//constraint->LuaRetain ( b );






		//btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
		//hingeC->setLowerLimit(-SIMD_HALF_PI * 0.5f);
		//hingeC->setUpperLimit( SIMD_HALF_PI * 0.5f);


	//btTypedConstraint*		pp;
	//pp = hingeC;


	//self->mWorld->addConstraint(hingeC, true);
	return 1;
}

//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointCone ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUNNNNNN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	btTransform localA, localB;
	localA.setIdentity(); 
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(a_x), btScalar(a_y), btScalar(a_z)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(b_x), btScalar(a_y), btScalar(a_z)));

	btConeTwistConstraint*	coneC;
	coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
	coneC = new btConeTwistConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB); //NEED TO MAKE JOINT SIZE
	coneC->setDbgDrawSize(0);

	coneC->setBreakingImpulseThreshold(100.0f);

	//self->mWorld->addConstraint(coneC, true);
	return 1;
};
//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointSlider ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUNNNNNN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	bool joint_bool = state.GetValue < bool >( 10, false );

	btTransform localA, localB;
	localA.setIdentity(); 
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(a_x), btScalar(a_y), btScalar(a_z)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(b_x), btScalar(a_y), btScalar(a_z)));

	btSliderConstraint*	sliderC;	
	sliderC = new btSliderConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB,joint_bool); //ANOTHER ARGGUMENT
	sliderC->setDbgDrawSize(0);
	sliderC->setBreakingImpulseThreshold(100.0f);

		sliderC->setLowerLinLimit(-15.0F);
		sliderC->setUpperLinLimit(-5.0F);
		sliderC->setLowerAngLimit(-SIMD_PI / 3.0F);
		sliderC->setUpperAngLimit( SIMD_PI / 3.0F);



	//self->mWorld->addConstraint(sliderC, true);
	return 1;
};
//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointFreedom ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUNNNNNN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	bool joint_bool = state.GetValue < bool >( 10, false );

	btTransform localA, localB;
	localA.setIdentity(); 
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(a_x), btScalar(a_y), btScalar(a_z)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(b_x), btScalar(a_y), btScalar(a_z)));

	btGeneric6DofConstraint*	freeDomeC;	
	freeDomeC = new btGeneric6DofConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB,joint_bool); //ANOTHER ARGGUMENT
	freeDomeC->setDbgDrawSize(0);
	freeDomeC->setBreakingImpulseThreshold(100.0f);

	btTransform sliderTransform;
	btVector3 lowerSliderLimit = btVector3(-10,0,0);
	btVector3 hiSliderLimit = btVector3(10,0,0);

	freeDomeC->setLinearLowerLimit(lowerSliderLimit);
	freeDomeC->setLinearUpperLimit(hiSliderLimit);
	freeDomeC->setAngularLowerLimit(btVector3(-SIMD_PI,0,0));
	freeDomeC->setAngularUpperLimit(btVector3(1.5,0,0));

	freeDomeC->getTranslationalLimitMotor()->m_enableMotor[0] = true;
	freeDomeC->getTranslationalLimitMotor()->m_targetVelocity[0] = -5.0f;
	freeDomeC->getTranslationalLimitMotor()->m_maxMotorForce[0] = 0.1f;


//btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*pBodyA, *pBodyB, frameInA, frameInB, true);
//pGen6DOFSpring->setLinearUpperLimit(btVector3(5., 0., 0.));
//pGen6DOFSpring->setLinearLowerLimit(btVector3(-5., 0., 0.));
//pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
//pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.5f));
//m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);
//pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));
//pGen6DOFSpring->enableSpring(0, true);
//pGen6DOFSpring->setStiffness(0, 39.478f);
//pGen6DOFSpring->setDamping(0, 0.5f);
//pGen6DOFSpring->enableSpring(5, true);
//pGen6DOFSpring->setStiffness(5, 39.478f);
//pGen6DOFSpring->setDamping(0, 0.3f);
//pGen6DOFSpring->setEquilibriumPoint();




//	self->mWorld->addConstraint(freeDomeC, true);
	return 1;
};


//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointPoint ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletConstraint, "UUUNNNNNN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	btVector3 pivotInA(a_x,a_y,a_z);
	btVector3 pivotInB(b_x,b_y,b_z);

	btPoint2PointConstraint*	PointC;	
	PointC = new btPoint2PointConstraint(*bodyA->mBody, *bodyB->mBody, pivotInA, pivotInB); //ANOTHER ARGGUMENT
	PointC->setDbgDrawSize(0);
	PointC->setBreakingImpulseThreshold(100.0f);
	//self->mWorld->addConstraint(PointC, true);
	return 1;
};


//----------------------------------------------------------------//
//SHOULD USE CONTRAINT CLASS NOT INDUVUAL CLASSES
int MOAIBulletConstraint::_newJointFixed ( lua_State* L ) {
	//MOAI_LUA_SETUP ( MOAIBulletConstraint, "UUUNNNNNN" )	

	MOAILuaState state ( L );
	if ( !state.CheckParams ( 1, "UUNNNNNN" )) return 0;

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	btTransform localA, localB;
	localA.setIdentity(); 
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(a_x), btScalar(a_y), btScalar(a_z)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(b_x), btScalar(a_y), btScalar(a_z)));

	btFixedConstraint*	fixedC;	
	fixedC = new btFixedConstraint(*bodyA->mBody, *bodyB->mBody, localA, localB); //ANOTHER ARGGUMENT
	fixedC->setDbgDrawSize(0);
	fixedC->setBreakingImpulseThreshold(100.0f);
	//self->mWorld->addConstraint(fixedC, true);


	return 1;
};




//================================================================//
// MOAICpConstraint
//================================================================//

//----------------------------------------------------------------//
void MOAIBulletConstraint::addToWorld (btDiscreteDynamicsWorld* mWorld ) {

	assert ( space );
	assert ( this->mConstraint );
	//cpSpaceAddConstraint ( space, this->mConstraint );
}

//----------------------------------------------------------------//
void MOAIBulletConstraint::removeFromWorld ( btDiscreteDynamicsWorld* mWorld ) {

	assert ( space );
	assert ( this->mConstraint );
	//cpSpaceRemoveConstraint ( space, this->mConstraint );
	

}

//----------------------------------------------------------------//
MOAIBulletConstraint::MOAIBulletConstraint () :
	mConstraint ( 0 ) {
	
	RTTI_BEGIN
	//	RTTI_EXTEND ( MOAICpPrim )
	RTTI_END
}

//----------------------------------------------------------------//
MOAIBulletConstraint::~MOAIBulletConstraint () {

	if ( this->mConstraint ) {
		//this->LuaRelease (( MOAICpBody* )this->mConstraint->a->data );
		//this->LuaRelease (( MOAICpBody* )this->mConstraint->b->data );
		//cpConstraintFree ( this->mConstraint );
	}
}

//----------------------------------------------------------------//
void MOAIBulletConstraint::RegisterLuaClass ( MOAILuaState& state ) {

	luaL_Reg regTable [] = {

		{ "newJointHinge",				_newJointHinge }, //isCLass
		{ "newJointCone",				_newJointCone }, //isCLass
		{ "newJointFixed",				_newJointFixed }, //isCLass
		{ "newJointPoint",				_newJointPoint }, //isCLass
		{ "newJointSlider",				_newJointSlider }, //isCLass
		{ "newJointFreedom",			_newJointFreedom }, //isCLass

		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAIBulletConstraint::RegisterLuaFuncs ( MOAILuaState& state ) {
	
	luaL_Reg regTable [] = {
//		{ "getBiasCoef",				_getBiasCoef },
//		{ "getMaxBias",					_getMaxBias },
//		{ "getMaxForce",				_getMaxForce },
//		{ "setBiasCoef",				_setBiasCoef },
//		{ "setMaxBias",					_setMaxBias },
//		{ "setMaxForce",				_setMaxForce },
		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}
