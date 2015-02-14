#ifndef	MOAIBULLETWHEEL_H
#define	MOAIBULLETWHEEL_H

class MOAIBulletWheel :
	public MOAIBulletVehicle {
private:
	//m_tuning,
	btRaycastVehicle *mVehicle;


	btVector3 mConnect;
	btVector3 mWheelDirection;
	btVector3 mWheelAxle;

	float mSuspensionRestLength;
	float mWheelRadius;	
	bool  mIsFrontWheel;


public:	



	DECL_LUA_FACTORY ( MOAIBulletWheel )	

	static int		_addWheelToVehicle			( lua_State* L );

	static int		_setConnectionPoint			( lua_State* L );
	static int		_setWheelDirection			( lua_State* L );
	static int		_wheelAxle					( lua_State* L );
	static int		_setWheelAxle				( lua_State* L );
	static int		_setWheelRadius				( lua_State* L );
	static int		_setFrontWheel				( lua_State* L );
	static int		_suspensionRestLength		( lua_State* L );


	//----------------------------------------------------------------//
					MOAIBulletWheel		();
					~MOAIBulletWheel		();
	void			RegisterLuaClass			( MOAILuaState& state );
	void			RegisterLuaFuncs			( MOAILuaState& state );


	void				setVehicle				(btRaycastVehicle *vehicle);
	void				setTune					(btDiscreteDynamicsWorld* world);




};

#endif
