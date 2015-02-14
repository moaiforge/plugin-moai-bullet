// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETTRANSFORM_H
#define	MOAIBULLETTRANSFORM_H

#include <moai-bullet/MOAIBulletWorld.h>
#include <bullet/src/LinearMath/BtIdebugDraw.h>
#include <bullet/src/btBulletDynamicsCommon.h>

class btTransform;

class MOAIBulletTransform : 
	public virtual MOAILuaObject {	
protected:	

	btTransform* mTransform;

	//----------------------------------------------------------------//
	static int			_destroy				( lua_State* L );
	static int			_setIdentity			( lua_State* L );
	static int			_setOrigin				( lua_State* L );
	static int			_setEulerZYX			( lua_State* L );
	static int			_doOffset				( lua_State* L );
	static int			_getScale				( lua_State* L );



public:
	
	friend class MOAIBulletWorld;
	friend class MOAIBulletBody;

	DECL_LUA_FACTORY ( MOAIBulletTransform )
	//----------------------------------------------------------------//
	void				Destroy					();
						MOAIBulletTransform		();
						~MOAIBulletTransform		();
	void				RegisterLuaClass		( MOAILuaState& state );
	void				RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
