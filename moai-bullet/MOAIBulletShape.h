#ifndef	MOAIBULLETSHAPE_H
#define	MOAIBULLETSHAPE_H

#include <moai-bullet/MOAIBulletWorld.h>

#include <bullet/src/LinearMath/btMotionState.h>
#include <bullet/src/btBulletDynamicsCommon.h>

class MOAIBulletShape :
	public MOAIBulletPrim,
	public virtual MOAILuaObject 
{
private:	

//WORLD
	btCollisionShape*		mShape;
	btCompoundShape*		mCompound;
	btRigidBody*			mBody;	
	
//SHOULD BE VEC3
	float mRot_x;
	float mRot_y;
	float mRot_z;

	float mLoc_x;
	float mLoc_y;
	float mLoc_z;

//US MOAI OR BULLET VECTOR
	ZLVec3D *mRot;
	ZLVec3D *mLoc;

	static int		_makePlane				( lua_State* L );
	static int		_makeSphere				( lua_State* L );
	static int		_makeBox				( lua_State* L );
	static int		_makeCylinder			( lua_State* L );
	static int		_makeCapsule			( lua_State* L );
	static int		_makeCone				( lua_State* L );


	static int		_addToBody				( lua_State* L );

	static int		_setMass				( lua_State* L );
	static int		_setMargin				( lua_State* L );

	static int		_SetPosition			( lua_State* L );
	static int		_SetRotation			( lua_State* L );

public:
	
	friend class MOAIBulletBody;
	DECL_LUA_FACTORY ( MOAIBulletShape )
	//----------------------------------------------------------------//
	void			setCompound					(btCompoundShape*	mCompound);
	void			setBody						(btRigidBody*		mBody);
	void			setOrigin					(float loc_x,float loc_y,float loc_z);
	void			setEulerZYX					(float rot_x,float rot_y,float rot_z);

	void			Destroy					();
					MOAIBulletShape			();
					~MOAIBulletShape		();
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );
};

#endif
