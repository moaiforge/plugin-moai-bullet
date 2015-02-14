#ifndef	MOAIBULLETWORLD_H
#define	MOAIBULLETWORLD_H


#include <bullet/src/btBulletDynamicsCommon.h>
#include <bullet/src/LinearMath/BtIdebugDraw.h>

//#include <bullet/src/LinearMath/btQuickprof.h> // #ifndef BT_NO_PROFILE-
//#include <moai-bullet/btRigidBodyWithCollisionEvents.h> //CUSTOM CALL BACKS DON"TWORK

//class MOAISim;

class MOAIBulletDebugDraw;
class MOAIBulletBody;
class MOAIBulletShape;
class MOAIBulletJoint;
class MOAIBulletArbiter;
class MOAIBulletWorld;


class MOAIBulletPrim :
	public virtual MOAILuaObject  {
protected:


	MOAIBulletWorld* mWorld;
	
	bool			mDestroy;
	MOAIBulletPrim*	mDestroyNext;	
public:
	friend class MOAIBulletWorld;

	GET_SET ( MOAIBulletWorld*, World, mWorld )

	//----------------------------------------------------------------//
	virtual void	Destroy					() = 0;
	float			GetUnitsToMeters		();
					MOAIBulletPrim			();
	
	//----------------------------------------------------------------//
	inline bool IsDestroyed () {
		return this->mDestroy;
	}
};


class MOAIBulletWorld : 
	public MOAIAction,
	//public ICollisionEvents,
	public virtual MOAILuaObject {		



private:


MOAILuaSharedPtr < MOAIBulletArbiter > mArbiter;


btDefaultCollisionConstructionInfo mConstructionInfo;
//Bullet collision debug-draw
MOAIBulletDebugDraw* mDebugDraw;
/// Bullet collision configuration.
btCollisionConfiguration* mCollisionConfiguration;
/// Bullet collision dispatcher.
btCollisionDispatcher* mCollisionDispatcher;
/// Bullet collision broadphase.
btBroadphaseInterface* mBroadphase;
/// Bullet constraint solver.
btConstraintSolver* mSolver;
/// Bullet physics world.
btDiscreteDynamicsWorld* mWorld;

MOAIBulletPrim*		mDestroyBodies;
MOAIBulletPrim*		mDestroyShapes; //SHAPES LIKE THIS, MAYBE NOT BECAUSE MANY SHAPES ON ONE BODY?
MOAIBulletPrim*		mDestroyJoints;

float mStep;
int   mMaxSubSteps; 
float mDrawScale;
float mDrawJointSize;
bool  mLock;

//----------------------------------------------------------------//
static int		_DrawDebugLua		    ( lua_State* L );




static int		_create					( lua_State* L );

static int		_defaultMaxCollisionAlgorithmPoolSize 			( lua_State* L );
static int		_defaultMaxPersistentManifoldPoolSize 			( lua_State* L );

static int		_setStep 				( lua_State* L );
static int		_setMaxSubSteps			( lua_State* L );

static int		_setDrawScale 			( lua_State* L );
static int		_setDrawJointSize		( lua_State* L );
static int		_setForceUpdateAllAabbs ( lua_State* L );
static int		_setGravity 			( lua_State* L );
static int		_useContinuous 			( lua_State* L );
static int		_enableSPU 				( lua_State* L );
static int		_allowedCcdPenetration 	( lua_State* L );


static int		_splitImpulse 			( lua_State* L );
static int		_Iterations 			( lua_State* L );

static int		_newBody				( lua_State* L );


static int		_addJointHinge			( lua_State* L );
static int		_addJointCone			( lua_State* L );
static int		_addJointFixed			( lua_State* L );
static int		_addJointFreedom		( lua_State* L );
static int		_addJointPoint			( lua_State* L );
static int		_addJointSlider			( lua_State* L );

void			Destroy					();
void			SayGoodbye				(btCompoundShape* shape ); 
void			SayGoodbye				(btTypedConstraint* joint );

void			ScheduleDestruction		( MOAIBulletBody& body );
void			ScheduleDestruction		( MOAIBulletShape& shape );
void			ScheduleDestruction		( MOAIBulletJoint& joint );

public:
	
	friend class MOAIBulletBody; 
	friend class MOAIBulletShapes;
	friend class MOAIBulletJoint;



	DECL_LUA_FACTORY ( MOAIBulletWorld )	
	//----------------------------------------------------------------//
	MOAIBulletWorld				();
	~MOAIBulletWorld			();

	void			setBody	();
	void			OnUpdate				( float step );


	bool			IsDone					();
	void			DrawDebug				();
	void			RegisterLuaClass	( MOAILuaState& state );
	void			RegisterLuaFuncs	( MOAILuaState& state );

	static bool callbackFunc(btManifoldPoint& cp,const btCollisionObject* obj1,int id1,int index1,const btCollisionObject* obj2,int id2,int index2);
	static void mNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);

// ICollisionEvents
//virtual bool	needsCollision(btCollisionObject* body0,btCollisionObject* body1);
//virtual bool	needsResponse(btCollisionObject* body0,btCollisionObject* body1);
//virtual void	dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher);

//protected :
//virtual void  OnCollisionStart(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);
//virtual void  OnCollisionContinue(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);
//virtual void  OnCollisionStop(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);	





};


#endif
