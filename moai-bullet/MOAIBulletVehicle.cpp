// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <bullet/src/btBulletDynamicsCommon.h>
#include <moai-bullet/MOAIBulletBody.h>
#include <moai-bullet/MOAIBulletVehicle.h>
#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletWheel.h>

//WHEELS NEED TO BE A SUB-CLASS
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
////**************************************************************************************************************
//btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
//{
//
//		btVector3 localInertia(0,0,0);
//		shape->calculateLocalInertia(mass,localInertia);
//
//		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
//		btRigidBody* body = new btRigidBody(rbInfo);
//
//		//body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
//		//body->setActivationState(DISABLE_DEACTIVATION);
//
//
//	return body;
//}
//
//
//
//int MOAIBulletBody::_AddCar ( lua_State* L ) {
//MOAI_LUA_SETUP ( MOAIBulletBody, "UN" )	
//
//
//	
////TEMP GLOBAL
//btRaycastVehicle	*m_vehicle;
//btRigidBody			*m_carChassis;
////MAKE A WHEEL SHAPE : JUST FOR DRAWING
////btCollisionShape* m_wheelShape;		
////m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
////m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
//
//btScalar suspensionRestLength(0.6);
//#define CUBE_HALF_EXTENTS 1
//
//float	gEngineForce			= 00.f;
//float	gBreakingForce			= 00.f;
//float	maxEngineForce			= 1000.f;//this should be engine/velocity dependent
//float	maxBreakingForce		= 100.f;
//float	gVehicleSteering		= 0.f;
//float	steeringIncrement		= 0.01f;
//float	steeringClamp			= 0.3f;


//float	wheelRadius				= 0.7f;
//float	wheelWidth				= 0.4f;
//float	wheelFriction			= 1000;//BT_LARGE_FLOAT;
//float	suspensionStiffness		= 20.f;
//float	suspensionDamping		= 2.3f;
//float	suspensionCompression	= 4.4f;
//float	rollInfluence			= 0.1f;//1.0f;
//
//btVector3 wheelDirectionCS0(0,-1,0);
//btVector3 wheelAxleCS(-1,0,0);
//
//
//
//btCompoundShape*	compound	 = new btCompoundShape(); //COMES FROM BODY
//
//
////********************************************************
////ADD THE CHILD SHAPES FOR CHASE TO COMMPUND
//
//			btTransform localTrans;
//			localTrans.setIdentity();
//			localTrans.getBasis().setEulerZYX(0,M_PI_2*2,0);
//			localTrans.setOrigin(btVector3(0,1,0));	
//	
//			btCollisionShape*	chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f)); //IS ATTACHED ALLREADY
//			compound->addChildShape(localTrans,chassisShape);
//
//
////********************************************************
////ADD THE CHILD SHAPES FOR CHASE TO COMMPUND
//
//			btTransform tr;
//			tr.setIdentity();
//			tr.setOrigin(btVector3(0,0.f,0));
//			tr.getBasis().setEulerZYX(0,M_PI_2*2,0);
//			m_carChassis = localCreateRigidBody(300,tr,compound);
//
//
//
////**************************************************************************
////**************************************************************************
////**************************************************************************
////**************************************************************************
//
//
//			//MAKE A RAY CAST
//			btVehicleRaycaster* m_vehicleRayCaster  = new btDefaultVehicleRaycaster(self->mWorld);
//			//TUNE IT
//			btRaycastVehicle::btVehicleTuning m_tuning;
//
//					//m_tuning.m_frictionSlip			= 0;
//					//m_tuning.m_maxSuspensionForce		= 0;
//					//m_tuning.m_maxSuspensionTravelCm	= 0;
//					//m_tuning.m_suspensionCompression	= 0;
//					//m_tuning.m_suspensionDamping		= 0;
//					//m_tuning.m_suspensionStiffness	= 0;
//
//
//			//MAKE A VEHICLE : TUNE : BODY : CASTER
//			m_vehicle			= new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);		
//
//			m_carChassis->setActivationState(DISABLE_DEACTIVATION);	
//
//			//SET VIEHICLE TO WORLD
//			self->mWorld->addVehicle(m_vehicle);		
//
//			//SET CORDINATES ON VEICL
//			int rightIndex = 0;
//			int upIndex = 1;
//			int forwardIndex = 2;
//			
//			m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);
//
//
//			// MAKE JOINTS
//			btVector3 connectionPointCS0;
//			float connectionHeight	= 1.2f;
//			bool  isFrontWheel		= false;	
//
//			//***********************
//				connectionPointCS0= btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);			
//					m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
//			//***********************
//				connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
//				m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
//			//***********************
//				connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);	
//				m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
//			//***********************
//				connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
//				m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
//
//			//DAMPING 
					//for (int i=0;i<m_vehicle->getNumWheels();i++)
					//{
					//	btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
					//	wheel.m_frictionSlip				= wheelFriction;

					//	wheel.m_suspensionStiffness			= suspensionStiffness;
					//	wheel.
					//	wheel.m_wheelsDampingRelaxation		= suspensionDamping;
					//	wheel.m_wheelsDampingCompression	= suspensionCompression;

					//	wheel.m_rollInfluence				= rollInfluence;
					//}
//		
//					//m_tuning.m_frictionSlip			= 0;
//					//m_tuning.m_maxSuspensionForce		= 0;
//					//m_tuning.m_maxSuspensionTravelCm	= 0;
//					//m_tuning.m_suspensionCompression	= 0;
//					//m_tuning.m_suspensionDamping		= 0;
//					//m_tuning.m_suspensionStiffness	= 0;
//
//
//
//
//
//
////************************************************************
////SET IT TO THE WORLD
//	self->mWorld->addRigidBody(m_carChassis);
//
//
//};


//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************


//----------------------------------------------------------------//
int MOAIBulletVehicle::_destroy ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "U" )
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletVehicle::_newWheel ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "U" )
	MOAIBulletWheel* wheel = new MOAIBulletWheel ();
	wheel->setVehicle(self->mRaycastVehicle);	
	self->LuaRetain ( wheel );
	wheel->PushLuaUserdata ( state );
	return 1;
};

//----------------------------------------------------------------//
int MOAIBulletVehicle::_applyGas ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "UN" )
	float gas		= state.GetValue < float >( 2, 0 );
	int wheel		= state.GetValue < int >( 3, 0 );	
	self->mRaycastVehicle->applyEngineForce(gas,wheel-1);
	return 0;
};

//----------------------------------------------------------------//
int MOAIBulletVehicle::_applyBrake ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "UN" )
	float brake		= state.GetValue < float >( 2, 0 );
	int wheel		= state.GetValue < int >( 3, 0 );
	self->mRaycastVehicle->setBrake(brake,wheel-1);
	return 0;
};

//----------------------------------------------------------------//
int MOAIBulletVehicle::_applySteering( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "UN" )
	float steer		= state.GetValue < float >( 2, 0 );
	int wheel		= state.GetValue < int >( 3, 0 );
	self->mRaycastVehicle->setSteeringValue(steer,wheel-1);
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletVehicle::_setCoordinateSystem ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "UNNN" )	
		//int rightIndex = 0;
		//int upIndex = 1;
		//int forwardIndex = 2;		

		int rightIndex		= state.GetValue < int >( 2, 0 );
		int upIndex			= state.GetValue < int >( 3, 0 );
		int forwardIndex	= state.GetValue < int >( 4, 0 );		

		btRaycastVehicle* vehicle = ( btRaycastVehicle* )self->mRaycastVehicle;
		vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

	return 0;
};
//----------------------------------------------------------------//
//SHOLD ME INDIVUAL

int MOAIBulletVehicle::_setTune ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "UNNN" )	
	//self->mTuning.m_frictionSlip				= state.GetValue < btScalar >( 2, 0.0f );
	//self->mTuning.m_maxSuspensionForce			= state.GetValue < btScalar >( 3, 0.0f );
	//self->mTuning.m_maxSuspensionTravelCm		= state.GetValue < btScalar >( 4, 0.0f );
	//self->mTuning.m_suspensionCompression		= state.GetValue < btScalar >( 5, 0.0f );
	//self->mTuning.m_suspensionDamping			= state.GetValue < btScalar >( 6, 0.0f );
	//self->mTuning.m_suspensionStiffness			= state.GetValue < btScalar >( 7, 0.0f );

	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletVehicle::_setToWorld ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletVehicle, "U" )	
	btRaycastVehicle::btVehicleTuning mTuning;	
	self->mRaycastVehicle	= new btRaycastVehicle(mTuning,self->mBody,self->mVehicleRayCaster);
	self->mWorld->addVehicle(self->mRaycastVehicle);	
	return 0;
};
//----------------------------------------------------------------//
void MOAIBulletVehicle::Destroy () {
	//if ( this->mCaster ) {	
	//};
};
//----------------------------------------------------------------//
MOAIBulletVehicle::MOAIBulletVehicle () :
	mVehicleRayCaster ( 0 ), //RAYCAST
	mBody		 ( 0 ),	
	mCompound	 ( 0 )
	{	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAILuaObject )
	RTTI_END
}
//----------------------------------------------------------------//
MOAIBulletVehicle::~MOAIBulletVehicle () {
	this->Destroy ();
}
//----------------------------------------------------------------//
void MOAIBulletVehicle::RegisterLuaClass ( MOAILuaState& state ) {
	MOAIBulletPrim::RegisterLuaClass ( state );
}
//----------------------------------------------------------------//
void MOAIBulletVehicle::RegisterLuaFuncs ( MOAILuaState& state ) {
	MOAIBulletPrim::RegisterLuaFuncs ( state );

	luaL_Reg regTable [] = {
		{ "destroy",						_destroy },
		{ "newWheel",						_newWheel },
		{ "setCoordinateSystem",			_setCoordinateSystem },
		{ "setTune",						_setTune },
		{ "setToWorld",						_setToWorld },

		{ "applyBrake",						_applyBrake },
		{ "applyGas",						_applyGas },
		{ "applySteering",					_applySteering },

		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}

//***************************************************************************
//***************************************************************************
//SET POINTERS
//----------------------------------------------------------------//
void MOAIBulletVehicle::setCompound (btCompoundShape*	mCompound) {
	this->mCompound = mCompound;
};
//----------------------------------------------------------------//
//REMBERING THE BODY??
void MOAIBulletVehicle::setBody (btRigidBody*		mBody) {
	this->mBody = mBody;
};
//----------------------------------------------------------------//
void MOAIBulletVehicle::setCaster (btVehicleRaycaster*	mCaster) {
	this->mVehicleRayCaster = mCaster;
};
//----------------------------------------------------------------//
void MOAIBulletVehicle::setWorld (btDiscreteDynamicsWorld* world) {
	this->mWorld = world;
};




