// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINTFREEDOM_H
#define	MOAIBULLETJOINTFREEDOM_H

#include <moai-bullet/MOAIBulletJoint.h>
class MOAIBulletJointFreedom :
	public MOAIBulletJoint {
private:
public:	
	DECL_LUA_FACTORY ( MOAIBulletJointFreedom )	

	static int		_setLinLimit					( lua_State* L );
	static int		_setAngLimit					( lua_State* L );
	//----------------------------------------------------------------//
					MOAIBulletJointFreedom		();
					~MOAIBulletJointFreedom		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );
};

#endif
