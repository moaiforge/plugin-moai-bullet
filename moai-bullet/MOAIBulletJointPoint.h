// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINTPOINT_H
#define	MOAIBULLETJOINTPOINT_H

#include <moai-bullet/MOAIBulletJoint.h>
class MOAIBulletJointPoint :
	public MOAIBulletJoint {
private:
public:	
	DECL_LUA_FACTORY ( MOAIBulletJointPoint )	

	static int		_setLinLimit					( lua_State* L );
	static int		_setAngLimit					( lua_State* L );
	//----------------------------------------------------------------//
					MOAIBulletJointPoint		();
					~MOAIBulletJointPoint		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );
};

#endif
