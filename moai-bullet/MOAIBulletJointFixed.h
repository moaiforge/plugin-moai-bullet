// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINTFIXED_H
#define	MOAIBULLETJOINTFIXED_H

#include <moai-bullet/MOAIBulletJoint.h>
class MOAIBulletJointFixed :
	public MOAIBulletJoint {
private:
public:	
	DECL_LUA_FACTORY ( MOAIBulletJointFixed )	

	static int		_setLinLimit					( lua_State* L );
	static int		_setAngLimit					( lua_State* L );
	//----------------------------------------------------------------//
					MOAIBulletJointFixed		();
					~MOAIBulletJointFixed		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );
};

#endif
