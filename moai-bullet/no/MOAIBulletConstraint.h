// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETCONSTRAINT_H
#define	MOAIBULLETCONSTRAINT_H

#include <bullet/src/LinearMath/BtIdebugDraw.h>
#include <bullet/src/btBulletDynamicsCommon.h>


//#include <chipmunk/chipmunk.h>
//#include <moai-chipmunk/MOAICpSpace.h>

struct btTypedConstraint;

//================================================================//
// MOAICpConstraint
//================================================================//
/**	@name	MOAICpConstraint
	@text	Chipmunk Constraint.
*/
class MOAIBulletConstraint :
	public virtual MOAILuaObject {
//	,public MOAICpPrim {
private:

	btTypedConstraint*	mConstraint;	

//----------------------------------------------------------------//
static int		_newJointHinge			( lua_State* L );
static int		_newJointCone			( lua_State* L );
static int		_newJointFixed			( lua_State* L );
static int		_newJointFreedom		( lua_State* L );
static int		_newJointPoint			( lua_State* L );
static int		_newJointSlider			( lua_State* L );

	
	//----------------------------------------------------------------//
	void			addToWorld			(btDiscreteDynamicsWorld* mWorld );
	void			removeFromWorld		(btDiscreteDynamicsWorld* mWorld);

public:
	
	friend class MOAIBulletWorld;
	
	DECL_LUA_FACTORY ( MOAIBulletConstraint )
	
	//----------------------------------------------------------------//
					MOAIBulletConstraint		();
					~MOAIBulletConstraint		();
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
