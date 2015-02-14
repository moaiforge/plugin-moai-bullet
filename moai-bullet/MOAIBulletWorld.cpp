#include "pch.h"

//#include <bullet/src/btBulletDynamicsCommon.h>
//#include "btRigidBodyWithCollisionEvents.h"

//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
//#include "bullet/src/BulletCollision/CollisionShapes/btBoxShape.h"
//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btPointCollector.h"
//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
//#include "bullet/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"

////*********************************************
//struct bulletObject{
//        int id;
//        float r,g,b;
//        bool hit;
//        btRigidBody* body;
//        bulletObject(btRigidBody* b,int i,float r0,float g0,float b0) : body(b),id(i),r(r0),g(g0),b(b0),hit(false) {}
//};
////*********************************************
//bool MOAIBulletWorld::callbackFunc(btManifoldPoint& cp,const btCollisionObject* obj1,int id1,int index1,const btCollisionObject* obj2,int id2,int index2)
//{
//        ((bulletObject*)obj1->getUserPointer())->hit=true;       
//       ((bulletObject*)obj2->getUserPointer())->hit=true;
//        return false;
//}
//

//WTF
#include <bullet/src/LinearMath/btQuickprof.h>



#include <moai-bullet/MOAIBulletWorld.h>
#include <moai-bullet/MOAIBulletDebugDraw.h>
#include <moai-bullet/MOAIBulletBody.h>
#include <moai-bullet/MOAIBulletShape.h>

#include <moai-bullet/MOAIBulletJoint.h>
#include <moai-bullet/MOAIBulletJointCone.h>
#include <moai-bullet/MOAIBulletJointFixed.h>
#include <moai-bullet/MOAIBulletJointFreedom.h>
#include <moai-bullet/MOAIBulletJointHinge.h>
#include <moai-bullet/MOAIBulletJointPoint.h>
#include <moai-bullet/MOAIBulletJointSlide.h>
#include <moai-bullet/MOAIBulletTransform.h>

// CProfileManager()
//THIS CALLBACKS DON' WORK I DON' UNDERSTAND
//#include "btRigidBodyWithCollisionEvents.h"
//btRigidBody* g_StaticGroundBox = NULL;
// ICollisionInterface
//void  MOAIBulletWorld::OnCollisionStart(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
//	printf("COLLID MOTHER FUCKER A\n");
//}
//void  MOAIBulletWorld::OnCollisionContinue(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
//	//printf("Collision Continue: %s - %s\n",thisBodyA!=g_StaticGroundBox ? thisBodyA->getCollisionShape()->getName() : "g_StaticGroundBox",bodyB!=g_StaticGroundBox ? bodyB->getCollisionShape()->getName() : "g_StaticGroundBox" );
//	printf("COLLID MOTHER FUCKER D\n");
//}
//void  MOAIBulletWorld::OnCollisionStop(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
//	//printf("Collision End: %s - %s\n",thisBodyA!=g_StaticGroundBox ? thisBodyA->getCollisionShape()->getName() : "g_StaticGroundBox",bodyB!=g_StaticGroundBox ? bodyB->getCollisionShape()->getName() : "g_StaticGroundBox" );
//	printf("COLLID MOTHER FUCKER C\n");
//}

//----------------------------------------------------------------//
float MOAIBulletPrim::GetUnitsToMeters () {
	if ( this->mWorld ) {
		//NO DONE
	}
	return 1.0f;
}
//----------------------------------------------------------------//
MOAIBulletPrim::MOAIBulletPrim () :
	mWorld ( 0 ),
	mDestroy ( false ),
	mDestroyNext ( 0 ) {
}
//----------------------------------------------------------------//
bool MOAIBulletWorld::IsDone () {
	return false;
}





//----------------------------------------------------------------//
struct YourOwnFilterCallback : public btOverlapFilterCallback
{
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
	{
		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);	

			btCollisionObject* colObj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
			btCollisionObject* colObj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
			
			//PREVENT COLLIDING WITH SELF
			if (colObj1 == colObj0) {
				return collides;
			};		

			//LOOK : THE PLANE THE PLANE THE FUCKING PLANE
			btCompoundShape *shapeA = (btCompoundShape *)colObj0->getCollisionShape();
			btCompoundShape *shapeB = (btCompoundShape *)colObj1->getCollisionShape();		
			if (shapeA->getNumChildShapes() > 0) {			
				if (shapeA->getChildShape(0)->getShapeType() == STATIC_PLANE_PROXYTYPE)  {
					return collides; //RETURN COLLID OR DOSNt HAPPEN
				};			
			};

			if (shapeB->getNumChildShapes() > 0) {
				if (shapeB->getChildShape(0)->getShapeType() == STATIC_PLANE_PROXYTYPE)  {
					return collides;//RETURN COLLID OR DOSNt HAPPEN
				};			
			};	

			//PUSH TO CALLBACK
			btRigidBody* rigBodyA = ( btRigidBody* )colObj0;
			MOAIBulletBody* moaiBodyA = ( MOAIBulletBody* )rigBodyA->getUserPointer (); 			
			
			btRigidBody* rigBodyB = ( btRigidBody* )colObj1;
			MOAIBulletBody* moaiBodyB = ( MOAIBulletBody* )rigBodyB->getUserPointer (); 			

				//if (moaiBodyA) {
				//	moaiBodyA->HandleCollision ( 1, moaiBodyA,moaiBodyB );
				//};

				if (moaiBodyB) {
					moaiBodyB->HandleCollision ( 1, moaiBodyB,moaiBodyA );
				};

		return collides;
	}
};
//----------------------------------------------------------------//
void MOAIBulletWorld::mNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
	btCollisionObject* colObj0 = static_cast<btCollisionObject*>(collisionPair.m_pProxy0->m_clientObject);
	btCollisionObject* colObj1 = static_cast<btCollisionObject*>(collisionPair.m_pProxy1->m_clientObject);

	if(dispatcher.needsCollision(colObj0,colObj1))
	{
		btCollisionObjectWrapper obj0Wrap(0,colObj0->getCollisionShape(),colObj0,colObj0->getWorldTransform(),-1,-1);
		btCollisionObjectWrapper obj1Wrap(0,colObj1->getCollisionShape(),colObj1,colObj1->getWorldTransform(),-1,-1);	
		
		//dispatcher will keep algorithms persistent in the collision pair
		if (!collisionPair.m_algorithm)
		{
			collisionPair.m_algorithm = dispatcher.findAlgorithm(&obj0Wrap,&obj1Wrap);
		}
		if (collisionPair.m_algorithm)
		{
			btManifoldResult contactPointResult(&obj0Wrap,&obj1Wrap);


			if (dispatchInfo.m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE)
			{
				//discrete collision detection query
				collisionPair.m_algorithm->processCollision(&obj0Wrap,&obj1Wrap,dispatchInfo,&contactPointResult);

				//CALL TO LUA
				btRigidBody* rigBodyA = ( btRigidBody* )colObj0;
				MOAIBulletBody* moaiBodyA = ( MOAIBulletBody* )rigBodyA->getUserPointer (); 
			
				//
				btRigidBody* rigBodyB = ( btRigidBody* )colObj1;
				MOAIBulletBody* moaiBodyB = ( MOAIBulletBody* )rigBodyB->getUserPointer (); 			

				moaiBodyB->HandleCollision ( 1, moaiBodyA,moaiBodyB );			}
			else
			{
				//continuous collision detection query, time of impact (toi)
				btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
				if (dispatchInfo.m_timeOfImpact > toi)
				{
					dispatchInfo.m_timeOfImpact = toi;
				}
			}
			if (contactPointResult.getPersistentManifold()->getNumContacts()>0) 
			{			


			
			}
		}
	}
	dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);

}
//----------------------------------------------------------------//

void MOAIBulletWorld::OnUpdate ( float step ) {			
     mWorld->stepSimulation( mStep, mMaxSubSteps);	

//UPDATE ALL OBJECTS
	for (int j=mWorld->getNumCollisionObjects()-1; j>=0 ;j--){	
		btCollisionObject* obj = mWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);

		//ACTIVE AND AWAKE THE SAME THING?
			if ( body->isActive()) {
				if (body && body->getMotionState())
				{			

					//CULLING PERHAPS ??
					//btTransform trans;
					//body->getMotionState()->getWorldTransform(trans);
					//printf("world pos = %f,%f,%f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
					
						//FIX FOR RAG DOLLS
						MOAIBulletBody* moaiBody = ( MOAIBulletBody* )body->getUserPointer (); //HAD TO ADD TO BULLET
						if (moaiBody) {
							moaiBody->ScheduleUpdate ();
						}
				};
			
			};	
	}


//***************************************************************************************************
//***************************************************************************************************
//***************************************************************************************************
//COLLISION MANIFOLDS
 /*  int numManifolds = mWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* obA =  const_cast<btCollisionObject*>(contactManifold->getBody0());
        btCollisionObject* obB =  const_cast<btCollisionObject*>(contactManifold->getBody1());

        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance()<0.f)
            {
				printf("objects \n");
		CALL TO LUA
				btRigidBody* rigBodyA = ( btRigidBody* )obA;
				btRigidBody* rigBodyB = ( btRigidBody* )obB;

				MOAIBulletBody* moaiBodyA = ( MOAIBulletBody* )rigBodyA->getUserPointer (); 				
				MOAIBulletBody* moaiBodyB = ( MOAIBulletBody* )rigBodyB->getUserPointer (); 			

				WHY IS I ON B
				moaiBodyB->HandleCollision ( 1, moaiBodyA,moaiBodyB );

				printf("COLLISION\n");
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;

            }
        }
    }*/
//***************************************************************************************************
//***************************************************************************************************
//***************************************************************************************************
};
//----------------------------------------------------------------//
void MOAIBulletWorld::DrawDebug () {	
	if ( this->mDebugDraw ) {	
		MOAIDraw::Bind ();	
		this->mDebugDraw->mSize = 0;

		MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get ();	
		gfxDevice.SetVertexMtxMode ( MOAIGfxDevice::VTX_STAGE_MODEL, MOAIGfxDevice::VTX_STAGE_PROJ );
		gfxDevice.SetVertexTransform ( MOAIGfxDevice::VTX_WORLD_TRANSFORM );	
		gfxDevice.BeginPrim ( ZGL_PRIM_LINES );

				this->mDebugDraw->mScale = this->mDrawScale; //* //(30.0f/10.0f);
				this->mWorld->debugDrawWorld();

	//	gfxDevice.addDrawCount();
		//gfxDevice.setPrimeSize(this->mDebugDraw->mSize*2);
		gfxDevice.EndPrim ();
	}
}
//----------------------------------------------------------------//
void MOAIBulletWorld::SayGoodbye ( btCompoundShape* shape) {

//***********************************************************
//NO USER DATA CONTAINER MUST ADDED IT TO BULLET SOURCE CODE??
//***********************************************************

		//MOAIBulletShape* moaiFixture = ( MOAIBulletShape* )fixture->getUserPointer ();
		//if ( moaiFixture->mFixture ) {
		//	moaiFixture->mFixture = 0;
		//	moaiFixture->SetWorld ( 0 );
		//	this->LuaRelease ( moaiFixture );
		//}
}

//----------------------------------------------------------------//
void MOAIBulletWorld::SayGoodbye ( btTypedConstraint* joint ) {

//***********************************************************
//NO USER DATA CONTAINER MUST ADDED IT TO BULLET SOURCE CODE??
//***********************************************************

	//MOAIBulletJoint* moaiJoint = ( MOAIBulletJoint* )joint->getUserPointer ();
	//if ( moaiJoint->mJoint ) {
	//		moaiJoint->mJoint = 0;
	//		moaiJoint->SetWorld ( 0 );
	//		this->LuaRelease ( moaiJoint );
	//}

}
//----------------------------------------------------------------//
void MOAIBulletWorld::ScheduleDestruction (MOAIBulletBody& body) {

	if ( !body.mDestroy ) {
		body.mDestroyNext = this->mDestroyBodies;
		this->mDestroyBodies = &body;
		body.mDestroy = true;
	}
	this->Destroy ();
}
//----------------------------------------------------------------//
void MOAIBulletWorld::ScheduleDestruction ( MOAIBulletShape& shape ) {

	if ( !shape.mDestroy ) {
		shape.mDestroyNext = this->mDestroyShapes;
		this->mDestroyShapes = &shape;
		shape.mDestroy = true;
	}
	this->Destroy ();
}
//----------------------------------------------------------------//
void MOAIBulletWorld::ScheduleDestruction (  MOAIBulletJoint& joint ) {

	if ( !joint.mDestroy ) {
		joint.mDestroyNext = this->mDestroyJoints;
		this->mDestroyJoints = &joint;
		joint.mDestroy = true;
	}
	this->Destroy ();
}
//----------------------------------------------------------------//
//I DON'T UNDERSTAND HOW THIS WORKS ???
void MOAIBulletWorld::Destroy () {

	if ( this->mLock ) return;
	this->mLock = true;

	while ( this->mDestroyShapes ) {
		MOAIBulletPrim* prim = this->mDestroyShapes;
		this->mDestroyShapes = this->mDestroyShapes->mDestroyNext;
		prim->Destroy ();
		
		prim->SetWorld ( 0 );
		this->LuaRelease ( prim );
	}
	
	while ( this->mDestroyJoints ) {
		MOAIBulletPrim* prim = this->mDestroyJoints;
		this->mDestroyJoints = this->mDestroyJoints->mDestroyNext;
		prim->Destroy ();
		
		prim->SetWorld ( 0 );
		this->LuaRelease ( prim );
	}
	
	while ( this->mDestroyBodies ) {
		MOAIBulletPrim* prim = this->mDestroyBodies;
		this->mDestroyBodies = this->mDestroyBodies->mDestroyNext;
		prim->Destroy ();
		
		prim->SetWorld ( 0 );
		this->LuaRelease ( prim );
	}	
	this->mLock = false;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_create( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )

	self->mBroadphase				= new btDbvtBroadphase();	
	self->mCollisionConfiguration	= new btDefaultCollisionConfiguration(self->mConstructionInfo);
	self->mCollisionDispatcher		= new btCollisionDispatcher(self->mCollisionConfiguration);
	self->mSolver					= new btSequentialImpulseConstraintSolver;
	self->mWorld					= new btDiscreteDynamicsWorld(self->mCollisionDispatcher, self->mBroadphase, self->mSolver, self->mCollisionConfiguration);

	//const btVector3 worldAabbMin( -1000.0, -1000.0, -1000.0 );
	//const btVector3 worldAabbMax( 1000.0, 1000.0, 1000.0 );
	//btBroadphaseInterface = new btAxisSweep3(worldAabbMin, worldAabbMax, 1638 );

	//GRAVITY
		self->mWorld->setGravity(btVector3(0, 0, 0));
	
	//SOLVER
		self->mWorld->getSolverInfo().m_splitImpulse			= true; // Disable by default for performance

	//DISPATCH
		self->mWorld->getDispatchInfo().m_useContinuous			= true;
		self->mWorld->getDispatchInfo().m_enableSPU				= true;
		self->mWorld->getDispatchInfo().m_allowedCcdPenetration = 0.01f;


	//self->mWorld->getDispatchInfo().

	//***********************************
	//OTHER STUFF
	//self->mWorld->setNumTasks

	self->mDebugDraw = new MOAIBulletDebugDraw ();
	self->mWorld->setDebugDrawer(self->mDebugDraw );		

	//****************************************************
	//SET CALLBACKS : DOSEN"T WORK
		//ICollisionEvents::SetInstance(this,this->mWorld);
		//gContactAddedCallback//=callbackFunc;

	//NEAR
		//mCollisionDispatcher->setNearCallback(this->mNearCallback);
		btOverlapFilterCallback * filterCallback = new YourOwnFilterCallback();
		self->mWorld->getPairCache()->setOverlapFilterCallback(filterCallback);

	return 1;
}

//----------------------------------------------------------------//
MOAIBulletWorld::MOAIBulletWorld () :
    mCollisionConfiguration(0),
    mCollisionDispatcher(0),
	mDebugDraw(0),
    mBroadphase(0),
    mSolver(0),
    mWorld(0),
	mDrawScale(1),
	mDrawJointSize(1),
	mStep(1/60),
	mMaxSubSteps(10),
	mLock(false)

{

RTTI_BEGIN
	RTTI_EXTEND ( MOAIAction )			
RTTI_END
//PRO-is wrapped in 
	//BT_NO_PROFILE 1
	//ProfileIterator * m_profileIterator =  CProfileManager::Get_Iterator();
	//LOWER MEMORY FOOT PRINT
	 this->mConstructionInfo		=  btDefaultCollisionConstructionInfo();
	//mConstructionInfo.m_defaultMaxCollisionAlgorithmPoolSize = 1023;
	//mConstructionInfo.m_defaultMaxPersistentManifoldPoolSize = 1023;	
	//m_defaultMaxPersistentManifoldPoolSize( 1638 ), //65536
	//m_defaultMaxCollisionAlgorithmPoolSize( 1638 ), //65536
	//m_defaultStackAllocatorSize( 131072 ) // 5 * 1024 * 1024
};
//----------------------------------------------------------------//
MOAIBulletWorld::~MOAIBulletWorld () {
	printf("\n~MOAIBulletWorld");

	if (this->mWorld) {
		delete mWorld;
	};

	if (this->mSolver) {
		delete mSolver;
	}

	if (this->mCollisionDispatcher) {
		delete mCollisionDispatcher;
	}

	if (this->mCollisionConfiguration) {
		delete mCollisionConfiguration;
	}

	if (this->mBroadphase) {
		delete mBroadphase;
	}

	if (this->mDebugDraw) {
		delete mDebugDraw;
	}
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_newBody ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UU" )	

	MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >(2, true );
	if ( !( transA )) return 0;
	btTransform ta = *transA->mTransform;

	//RESET VALUES	

	//NEW CLASS
		MOAIBulletBody* body = new MOAIBulletBody ();	
	//MASS
		btScalar mMass = 1.0;
	//INERTIA
		btVector3 mInertia(0.0, 0.0, 0.0);
	//MOTION & COMPOIND
		body->mMotion	= new btDefaultMotionState(ta);	
		body->mCompound = new btCompoundShape();   
	

	btRigidBody::btRigidBodyConstructionInfo info(mMass,body->mMotion,body->mCompound,mInertia);  

	//*********************************************************
	//body->mBody = new btRigidBody(info); //DOESN'T WORK
	//static_cast <btRigidBodyWithEvents*> (body->mBody)->setMonitorCollisions(true); 
	//static_cast <btRigidBodyWithEvents*>(body->mBody)->setMonitorCollisions(true);
	//body->mBody->setMonitorCollisions(true);	

	
	body->mBody = new btRigidBody(info);

	body->setWorld(self->mWorld);	
	body->SetBody(body->mBody); //SHOULDNT THIS BE MOAI DATA
	self->LuaRetain ( body );
	body->PushLuaUserdata ( state );
	return 1;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointHinge ( lua_State* L ) {

	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUUU" )	
	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );	

	if ( !( bodyA && bodyB )) return 0;

		MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >( 4, true );
		MOAIBulletTransform* transB = state.GetLuaObject < MOAIBulletTransform >( 5, true );

		if ( !( transA && transB )) return 0;

			btTransform ta = *transA->mTransform;
			btTransform tb = *transB->mTransform;

	btHingeConstraint*		hingeC;
	hingeC = new btHingeConstraint(*bodyA->mBody, *bodyB->mBody, ta, tb);

	hingeC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(hingeC, true);	

	MOAIBulletJointHinge* mJoint = new MOAIBulletJointHinge (); 

	mJoint->SetJoint(hingeC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);

	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );

	mJoint->PushLuaUserdata ( state );
	return 1;
}

//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointCone ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUUU" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;


		MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >( 4, true );
		MOAIBulletTransform* transB = state.GetLuaObject < MOAIBulletTransform >( 5, true );

		if ( !( transA && transB )) return 0;

			btTransform ta = *transA->mTransform;
			btTransform tb = *transB->mTransform;

	btConeTwistConstraint*	coneC;	
	coneC = new btConeTwistConstraint(*bodyA->mBody, *bodyB->mBody, ta, tb); 
	coneC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(coneC, true);	

	MOAIBulletJointCone* mJoint = new MOAIBulletJointCone (); 

	mJoint->SetJoint(coneC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);

	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );
	mJoint->PushLuaUserdata ( state );


	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointSlider ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUUUN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

		MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >( 4, true );
		MOAIBulletTransform* transB = state.GetLuaObject < MOAIBulletTransform >( 5, true );

		if ( !( transA && transB )) return 0;

			btTransform ta = *transA->mTransform;
			btTransform tb = *transB->mTransform;

	bool joint_bool = state.GetValue < bool >( 6, false );

	btSliderConstraint*	sliderC;	
	sliderC = new btSliderConstraint(*bodyA->mBody, *bodyB->mBody, ta, tb,joint_bool); 
	sliderC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(sliderC, true);

		//sliderC->setBreakingImpulseThreshold(100.0f);
		//sliderC->setLowerLinLimit(-15.0F);
		//sliderC->setUpperLinLimit(-5.0F);
		//sliderC->setLowerAngLimit(-SIMD_PI / 3.0F);
		//sliderC->setUpperAngLimit( SIMD_PI / 3.0F);
		//self->mWorld->addConstraint(sliderC, true);


	MOAIBulletJointSlide* mJoint = new MOAIBulletJointSlide (); 

	mJoint->SetJoint(sliderC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);

	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );
	mJoint->PushLuaUserdata ( state );

	
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointFreedom ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUUUN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

		MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >( 4, true );
		MOAIBulletTransform* transB = state.GetLuaObject < MOAIBulletTransform >( 5, true );

		if ( !( transA && transB )) return 0;

			btTransform ta = *transA->mTransform;
			btTransform tb = *transB->mTransform;


			bool joint_bool = state.GetValue < bool >( 5, false );

	btGeneric6DofConstraint*	freeDomeC;	
	freeDomeC = new btGeneric6DofConstraint(*bodyA->mBody, *bodyB->mBody, ta, tb,joint_bool); //ANOTHER ARGGUMENT
	freeDomeC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(freeDomeC, true);


//TODO: NEED TO ADD LUA CALL
	freeDomeC->setBreakingImpulseThreshold(100.0f);
	btTransform sliderTransform;
	btVector3 lowerSliderLimit = btVector3(-10,0,0);
	btVector3 hiSliderLimit = btVector3(10,0,0);

	freeDomeC->setLinearLowerLimit(lowerSliderLimit);
	freeDomeC->setLinearUpperLimit(hiSliderLimit);
	freeDomeC->setAngularLowerLimit(btVector3(-SIMD_PI,0,0));
	freeDomeC->setAngularUpperLimit(btVector3(1.5,0,0));

	freeDomeC->getTranslationalLimitMotor()->m_enableMotor[0] = true;
	freeDomeC->getTranslationalLimitMotor()->m_targetVelocity[0] = -5.0f;
	freeDomeC->getTranslationalLimitMotor()->m_maxMotorForce[0] = 0.1f;

	MOAIBulletJointFreedom* mJoint = new MOAIBulletJointFreedom (); 
	mJoint->SetJoint(freeDomeC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);
	
	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );
	mJoint->PushLuaUserdata ( state );


//***********************************************************************
//***********************************************************************

//btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*pBodyA, *pBodyB, frameInA, frameInB, true);
//pGen6DOFSpring->setLinearUpperLimit(btVector3(5., 0., 0.));
//pGen6DOFSpring->setLinearLowerLimit(btVector3(-5., 0., 0.));
//pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
//pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.5f));
//m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);
//pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));
//pGen6DOFSpring->enableSpring(0, true);
//pGen6DOFSpring->setStiffness(0, 39.478f);
//pGen6DOFSpring->setDamping(0, 0.5f);
//pGen6DOFSpring->enableSpring(5, true);
//pGen6DOFSpring->setStiffness(5, 39.478f);
//pGen6DOFSpring->setDamping(0, 0.3f);
//pGen6DOFSpring->setEquilibriumPoint();


	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointFixed ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUUUN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

		MOAIBulletTransform* transA = state.GetLuaObject < MOAIBulletTransform >( 4, true );
		MOAIBulletTransform* transB = state.GetLuaObject < MOAIBulletTransform >( 5, true );

		if ( !( transA && transB )) return 0;

			btTransform ta = *transA->mTransform;
			btTransform tb = *transB->mTransform;

	btFixedConstraint*	fixedC;	
	fixedC = new btFixedConstraint(*bodyA->mBody, *bodyB->mBody, ta, tb); 
	fixedC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(fixedC, true);

	MOAIBulletJointFixed* mJoint = new MOAIBulletJointFixed (); 
	mJoint->SetJoint(fixedC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);

	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );
	mJoint->PushLuaUserdata ( state );
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_addJointPoint ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UUUNNNNNN" )	

	MOAIBulletBody* bodyA = state.GetLuaObject < MOAIBulletBody >( 2, true );
	MOAIBulletBody* bodyB = state.GetLuaObject < MOAIBulletBody >( 3, true );
	
	if ( !( bodyA && bodyB )) return 0;

	float a_x = state.GetValue < float >( 4, 0.0f );
	float a_y = state.GetValue < float >( 5, 0.0f );
	float a_z = state.GetValue < float >( 6, 0.0f );

	float b_x = state.GetValue < float >( 7, 0.0f );
	float b_y = state.GetValue < float >( 8, 0.0f );
	float b_z = state.GetValue < float >( 9, 0.0f );

	btVector3 pivotInA(a_x,a_y,a_z);
	btVector3 pivotInB(b_x,b_y,b_z);

	btPoint2PointConstraint*	PointC;	
	PointC = new btPoint2PointConstraint(*bodyA->mBody, *bodyB->mBody, pivotInA, pivotInB); 
	PointC->setDbgDrawSize(self->mDrawJointSize);
	self->mWorld->addConstraint(PointC, true);

	MOAIBulletJointPoint* mJoint = new MOAIBulletJointPoint (); 
	mJoint->SetJoint(PointC);
	mJoint->SetBodyA(bodyA);
	mJoint->SetBodyB(bodyB);
	
	mJoint->SetWorld ( self );
	mJoint->LuaRetain ( bodyA );
	mJoint->LuaRetain ( bodyB );
	self->LuaRetain ( mJoint );

	mJoint->PushLuaUserdata ( state );
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_setStep ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )
	float step = state.GetValue < float >( 2, 1.0f/60.0f );
	self->mStep = step;	
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_setMaxSubSteps ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )
	int step = state.GetValue < int >( 2, 10 );
	self->mMaxSubSteps = step;	
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_setGravity ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )	
	float gravity_x = state.GetValue < float >( 2, 0.0f );
	float gravity_y = state.GetValue < float >( 3, 0.0f );
	float gravity_z = state.GetValue < float >( 4, 0.0f );
	self->mWorld->setGravity(btVector3(gravity_x, gravity_y, gravity_z));
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_setForceUpdateAllAabbs ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	bool forceUpdate = state.GetValue < bool >( 2, true );
	self->mWorld->setForceUpdateAllAabbs(forceUpdate);
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_defaultMaxCollisionAlgorithmPoolSize ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	u32 defaultMaxCollisionAlgorithmPoolSize = state.GetValue < u32 >( 2, true );
	self->mConstructionInfo.m_defaultMaxCollisionAlgorithmPoolSize = defaultMaxCollisionAlgorithmPoolSize;
	return 0;	
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_defaultMaxPersistentManifoldPoolSize ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	u32 defaultMaxCollisionAlgorithmPoolSize = state.GetValue < u32 >( 2, true );
	self->mConstructionInfo.m_defaultMaxPersistentManifoldPoolSize = defaultMaxCollisionAlgorithmPoolSize;
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletWorld::_useContinuous ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	bool continous = state.GetValue < bool >( 2, true );
	self->mWorld->getDispatchInfo().m_useContinuous	= continous;
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_enableSPU	( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	bool spu = state.GetValue < bool >( 2, true );
	self->mWorld->getDispatchInfo().m_enableSPU	= spu;
	return 0;
}

//----------------------------------------------------------------//
int MOAIBulletWorld::_allowedCcdPenetration	( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	float pen = state.GetValue < float >( 2, 0.0f );
	self->mWorld->getDispatchInfo().m_allowedCcdPenetration	= pen;
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_splitImpulse ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )
	bool impluse = state.GetValue < bool >( 2, true );
	///NOT SURE WHY EXMAPLE SHOWS BOOLEAN BECAUSE IT WANTS AN INTEGER
	self->mWorld->getSolverInfo().m_splitImpulse		= impluse;	
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_Iterations ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )	
	int iterations = state.GetValue < int >( 2, 4 );
	btContactSolverInfo& info = self->mWorld->getSolverInfo();
	info.m_numIterations = iterations;
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_setDrawScale ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )	
	float drawScale = state.GetValue < float >( 2, 1.0f );
	self->mDrawScale = drawScale;
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_setDrawJointSize ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "UN" )	
	float drawSize = state.GetValue < float >( 2, 1.0f );
	self->mDrawJointSize = drawSize;
	return 0;
}
//----------------------------------------------------------------//
int MOAIBulletWorld::_DrawDebugLua ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletWorld, "U" )	
	self->DrawDebug();	
	return 0;
}
//----------------------------------------------------------------//
void MOAIBulletWorld::RegisterLuaClass ( MOAILuaState& state ) {
	MOAIAction::RegisterLuaClass ( state );		
	state.SetField ( -1, "COMPOUND_SHAPE", ( u32 )1 );
}

//----------------------------------------------------------------//
void MOAIBulletWorld::RegisterLuaFuncs ( MOAILuaState& state ) {
	MOAIAction::RegisterLuaFuncs ( state );
	luaL_Reg regTable [] = {


//MANIFOLD
		{ "defaultMaxCollisionAlgorithmPoolSize",					_defaultMaxCollisionAlgorithmPoolSize }, 
		{ "defaultMaxPersistentManifoldPoolSize",					_defaultMaxPersistentManifoldPoolSize }, 
//INTI
		{ "create",						_create }, 
		{ "newBody",					_newBody }, 
//JOINTS
		{ "addJointHinge",				_addJointHinge },
		{ "addJointCone",				_addJointCone }, 
		{ "addJointFixed",				_addJointFixed },
		{ "addJointPoint",				_addJointPoint },
		{ "addJointSlider",				_addJointSlider },
		{ "addJointFreedom",			_addJointFreedom },

//WORLD
		{ "setDrawScale",				_setDrawScale },
		{ "setDrawJointSize",			_setDrawJointSize },
		{ "setForceUpdateAllAabbs",		_setForceUpdateAllAabbs },
		
		{ "setStep",					_setStep },
		{ "setMaxSubSteps",				_setMaxSubSteps },
	
		{ "setGravity",					_setGravity },
		{ "useContinuous",				_useContinuous },
		{ "splitImpulse",				_splitImpulse },

		{ "allowedCcdPenetration",		_allowedCcdPenetration },
		{ "enableSPU",					_enableSPU },

		{ "iterations",					_Iterations },	
//DEBUG DRAW
		{"drawDebugLua",				_DrawDebugLua	},

		{ NULL, NULL }
	};	
	luaL_register ( state, 0, regTable );
}




