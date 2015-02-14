// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIBULLETARBITER_H
#define	MOAIBULLETARBITER_H

#include <bullet/src/LinearMath/BtIdebugDraw.h>

// Forward declaration
class MOAIBulletWorld;


class MOAIBulletArbiter :
	public virtual MOAILuaObject {
	//,
	//public b2ContactListener {
private:

	//b2Contact*					mContact;
	//const b2ContactImpulse*		mImpulse;
	//b2Vec2  mContactNormal;
	//float	mNormalImpulse;
	//float	mTangentImpulse;


	const MOAIBulletWorld*       mWorld;

	//----------------------------------------------------------------//
	static int		_getContactNormal		( lua_State* L );
	static int		_getNormalImpulse		( lua_State* L );
	static int		_getTangentImpulse		( lua_State* L );
	static int		_setContactEnabled		( lua_State* L );
	
	//----------------------------------------------------------------//
			//void	BeginContact	( b2Contact* contact );
			//void	EndContact		( b2Contact* contact );
			//void	PostSolve		( b2Contact* contact, const b2ContactImpulse* impulse );
			//void	PreSolve		( b2Contact* contact, const b2Manifold* oldManifold );


			void	BeginContact	(  );
			void	EndContact		(  );
			void	PostSolve		(  );
			void	PreSolve		(  );



	//----------------------------------------------------------------//
	float   GetUnitsToMeters ( ) const;

public:
	
	DECL_LUA_FACTORY ( MOAIBulletArbiter )
	
	enum {
		BEGIN			= 0x00000001,
		END				= 0x00000002,
		POST_SOLVE		= 0x00000004,
		PRE_SOLVE		= 0x00000008,
		ALL				= 0x0000000f,
	};
	
	//----------------------------------------------------------------//
					MOAIBulletArbiter		();
					MOAIBulletArbiter        ( const MOAIBulletWorld &world );
					~MOAIBulletArbiter		();
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
