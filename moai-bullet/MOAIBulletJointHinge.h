// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINTHINGE_H
#define	MOAIBULLETJOINTHINGE_H

#include <moai-bullet/MOAIBulletJoint.h>
class MOAIBulletJointHinge :
	public MOAIBulletJoint {
private:
public:	
	DECL_LUA_FACTORY ( MOAIBulletJointHinge )	

	static int		_setLimit						( lua_State* L );
	static int		_setLinLimit					( lua_State* L );
	static int		_setAngLimit					( lua_State* L );
	//----------------------------------------------------------------//
					MOAIBulletJointHinge		();
					~MOAIBulletJointHinge		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );
};

#endif
