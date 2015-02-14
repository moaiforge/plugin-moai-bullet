// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETBODY_H
#define	MOAIBULLETBODY_H

#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletVehicle.h>

#include <bullet/src/LinearMath/btMotionState.h>
#include <bullet/src/btBulletDynamicsCommon.h>
#include <limits>

class btRigidBody;

typedef unsigned short  uint16;
typedef unsigned int	uint32;

//static const float DEFAULT_MASS = 0.0f;
//static const float DEFAULT_FRICTION = 0.5f;
//static const float DEFAULT_RESTITUTION = 0.0f;
//static const float DEFAULT_ROLLING_FRICTION = 0.0f;

static const uint16 DEFAULT_COLLISION_GROUP = 0x1;
static const uint16 DEFAULT_COLLISION_MASK = 0x1; //WTF?

#define MOAI_BULLET_idName	"idName"
#define MOAI_BULLET_idType	"idType"

class MOAIBulletBody :
	public MOAITransformBase,	
	public MOAIBulletPrim
{
private:

	MOAILuaMemberRef	mCollisionHandler;

	uint16 mCollision_group;
	uint16 mCollision_mask;
//WORLD
	btDiscreteDynamicsWorld* mWorld; 
//BODY
	btRigidBody*			mBody;

//MOTION
	btDefaultMotionState*	mMotion;

//SHAPE : SHOULD HAVE OWN SHAPES MAYBE? //btCollisionShape*		mShape;	
	btCompoundShape*		mCompound;

//FOR STATE SAVE
	btVector3	mRot;
	btVector3	mLoc;
	string		idName;

	static int		_SetIdType		 ( lua_State* L );
	static int		_SetIdName		 ( lua_State* L );
//----------------------------------------------------------------//
	static int		_CleanProxyFromPairs    ( lua_State* L );

	static int		_NewShape				( lua_State* L );
	static int		_NewVehicle				( lua_State* L );

	static int		_SetFilter				( lua_State* L );
	static int		_SetCallback			( lua_State* L );

	static int		_AddCollisionGroup		( lua_State* L );
	static int		_AddCollisionMask		( lua_State* L );
	static int		_SetCollisionFlags		( lua_State* L );
	static int		_NoResponse				( lua_State* L );
	
	static int		_AddToBody				( lua_State* L );

//STATE
	static int		_stateSet				( lua_State* L );
	static int		_stateRest				( lua_State* L );
//CLEAR FORCE
	static int		_clearForces			( lua_State* L );

//OBJECTS
	static int		_AddRag				( lua_State* L );
	static int		_AddCar				( lua_State* L );
	static int		_CarUpdate			( lua_State* L );

//LINEAR
	static int		_SetLinearVelocity			( lua_State* L );
	static int		_SetLinearFactor			( lua_State* L );
	static int		_SetLinearRestThreshold		( lua_State* L );
	static int		_SetLinearDamping			( lua_State* L );
//ANGULAR
	static int		_SetAngularVelocity				( lua_State* L );
	static int		_SetAngularFactor				( lua_State* L );
	static int		_SetAngularRestThreshold		( lua_State* L );
	static int		_SetAngularDamping				( lua_State* L );
//FRICTION
	static int		_SetFriction				( lua_State* L );
	static int		_SetAnisotropicFriction		( lua_State* L );
	static int		_SetRollingFriction			( lua_State* L );
//RESITUTION	
	static int		_SetRestitution			( lua_State* L );
//FORCE
	static int		_ApplyForce				( lua_State* L );
	static int		_ApplyForceOffset		( lua_State* L );
//TORQUE
	static int		_ApplyTorque			( lua_State* L );
//IMPULSE
	static int		_ApplyImpulse			( lua_State* L );
	static int		_ApplyImpulseOffset		( lua_State* L );
	static int		_ApplyTorqueImpulse		( lua_State* L );	
//RESET
	static int		_ResetForces			( lua_State* L );	
//SET GARVITY
	static int		_SetGravity				( lua_State* L );
//DAMPING
	static int		_SetDamping				( lua_State* L );
	static int		_SetDeactivationTime	( lua_State* L );
	static int		_SetSleepingThresholds	( lua_State* L );	

//POSTION
	static int		_SetPosition		( lua_State* L );
	static int		_SetRotation		( lua_State* L );

//WORLD
	static int		_AddToWorld			( lua_State* L );

//ACTIVITY
	static int		_SetKinematic		( lua_State* L );
	static int		_SetActivationState	( lua_State* L );

//SET
	static int		_SetCcdMotionThreshold		( lua_State* L );
	static int		_SetCcdSweptSphereRadius	( lua_State* L );
	
//GET
	static int		_GetPosition				( lua_State* L );
	static int		_GetRotation				( lua_State* L );
	static int		_GetLinearVelocity			( lua_State* L );
	static int		_GetLinearFactor			( lua_State* L );
	static int		_GetVelocityAtPoint			( lua_State* L );
	static int		_GetLinearRestThreshold		( lua_State* L );	
	static int		_GetLinearDamping			( lua_State* L );	
	static int		_GetAngularVelocity			( lua_State* L );
	static int		_GetAngularFactor			( lua_State* L );
	static int		_GetAngularRestThreshold	( lua_State* L );
	static int		_GetAngularDamping			( lua_State* L );
	static int		_GetFriction				( lua_State* L );
	static int		_GetAnisotropicFriction		( lua_State* L );
	static int		_GetRollingFriction			( lua_State* L );
	static int		_GetRestitution				( lua_State* L );
	static int		_GetContactProcessingThreshold	( lua_State* L );
	static int		_GetCcdRadius				( lua_State* L );
	static int		_GetCcdMotionThreshold		( lua_State* L );
	static int		_IsActive					( lua_State* L );
	static int		_GetCollidingBodies			( lua_State* L );
//REMOVE
	static int		_RemoveBodyFromWorld			( lua_State* L );

	//----------------------------------------------------------------//
	bool			ApplyAttrOp				( u32 attrID, MOAIAttrOp& attrOp, u32 op );
	void			SetBody					( btRigidBody* body );
	void			OnDepNodeUpdate			();

public:	
	friend class MOAIBulletShape;
	friend class MOAIBulletWorld;
	friend class MOAIBulletJoint;

	DECL_LUA_FACTORY ( MOAIBulletBody )
	//----------------------------------------------------------------//
	
	void			setWorld				(btDiscreteDynamicsWorld* world_);	
	void			HandleCollision			(u32 eventType, MOAIBulletBody* bodyA, MOAIBulletBody* bodyB);
	void			Destroy					();
					MOAIBulletBody			();
					~MOAIBulletBody			();
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );


	void			SetValue				(cc8* key );

	enum {
			EVENT_VALUE_CHANGED,
		};

	//template < typename TYPE >
	//void SetValue ( cc8* key, TYPE value ) {	
	//	MOAIScopedLuaState state = MOAILuaRuntime::Get ().State ();		
	//	state.Push ( key );
	//	state.Push ( value );		
	//	this->SetValue ( state );
	//}



};

#endif
