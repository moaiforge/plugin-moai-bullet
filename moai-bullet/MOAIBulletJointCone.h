// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINTCONE_H
#define	MOAIBULLETJOINTCONE_H

#include <moai-bullet/MOAIBulletJoint.h>
class MOAIBulletJointCone :
	public MOAIBulletJoint {
private:
public:	
	DECL_LUA_FACTORY ( MOAIBulletJointCone )	
	static int		_setLimit						( lua_State* L );
	static int		_setLinLimit					( lua_State* L );
	static int		_setAngLimit					( lua_State* L );
	//----------------------------------------------------------------//
					MOAIBulletJointCone			();
					~MOAIBulletJointCone		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );
};

#endif
