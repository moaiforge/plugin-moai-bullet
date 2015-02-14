// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETJOINT_H
#define	MOAIBULLETJOINT_H

#include <moai-bullet/MOAIBulletWorld.h>

class btTypedConstraint;

//================================================================//
// MOAIBulletJoint
//================================================================//
/**	@name MOAIBulletJoint
	@text bullet joint.
*/
class MOAIBulletJoint :
	public MOAIBulletPrim {
protected:
	
	btTypedConstraint*	mJoint;
	MOAIBulletBody*		mBodyA;
	MOAIBulletBody*		mBodyB;

	//----------------------------------------------------------------//
	static int			_destroy							( lua_State* L );
	static int			_getBodyA							( lua_State* L );
	static int			_getBodyB							( lua_State* L );
	static int			_setBreakingImpulse					( lua_State* L );
	static int			_getBreakingImpulse					( lua_State* L );
	static int			_isEnabled							( lua_State* L );
	static int			_setEnabled							( lua_State* L );	

	//----------------------------------------------------------------//
	void				SetJoint				( btTypedConstraint* joint );
	void				SetBodyA				( MOAIBulletBody* body );
	void				SetBodyB				( MOAIBulletBody* body );

public:
	
	friend class MOAIBulletWorld;
	friend class MOAIBulletJointSlide;
	friend class MOAIBulletJointHinge;
	friend class MOAIBulletJointCone;
	friend class MOAIBulletJointFixed;
	friend class MOAIBulletJointFreedom;
	friend class MOAIBulletJointPoint;

	//----------------------------------------------------------------//
	void				Destroy					();
						MOAIBulletJoint			();
						~MOAIBulletJoint		();
	void				RegisterLuaClass		( MOAILuaState& state );
	void				RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
