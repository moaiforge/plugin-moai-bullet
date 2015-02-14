// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETVEHICLE_H
#define	MOAIBULLETVEHICLE_H

#include <moai-bullet/MOAIBulletWorld.h>
//PRIME MAYBE NOT?
class MOAIBulletVehicle :
	public MOAIBulletPrim {
protected:

btRaycastVehicle				*mRaycastVehicle;
btVehicleRaycaster				*mVehicleRayCaster;
btCompoundShape					*mCompound;
btRigidBody						*mBody;	
btDiscreteDynamicsWorld			*mWorld;	

	//----------------------------------------------------------------//
	static int			_destroy							( lua_State* L );
	static int			_applyGas							( lua_State* L );
	static int			_applyBrake							( lua_State* L );
	static int			_applySteering						( lua_State* L );
	
	static int			_newWheel							( lua_State* L );
	static int			_setCoordinateSystem				( lua_State* L );
	static int			_setTune							( lua_State* L );
	static int			_setToWorld							( lua_State* L );
public:
	
	friend class MOAIBulletWorld;
	friend class MOAIBulletBody;

	
	DECL_LUA_FACTORY ( MOAIBulletVehicle )

	//----------------------------------------------------------------//
	//----------------------------------------------------------------//
	void				setCompound				(btCompoundShape*		mCompound);
	void				setBody					(btRigidBody*			mBody);
	void				setCaster				(btVehicleRaycaster*	mCaster);
	void				setWorld				(btDiscreteDynamicsWorld* world);


	void				Destroy					();
						MOAIBulletVehicle		();
						~MOAIBulletVehicle		();
	void				RegisterLuaClass		( MOAILuaState& state );
	void				RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
