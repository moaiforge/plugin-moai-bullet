/*
Created: 11/10/2009
*/

#pragma once
#ifndef BTRIGIDBODYWITHCOLLISIONEVENTS_H__
#define BTRIGIDBODYWITHCOLLISIONEVENTS_H__

#include <bullet/src/btBulletDynamicsCommon.h>
#include <bullet/src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>
#include <bullet/src/BulletDynamics/Dynamics/btActionInterface.h>

// USAGE:
/*
1) Inherit your application class from ICollisionEvents.
2) Add to your class the ICollisionEvents abstract methods:
	// ICollisionEvents
	virtual void  OnCollisionStart(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);
	virtual void  OnCollisionContinue(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);
	virtual void  OnCollisionStop(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse);	
	//--------------------
3) As soon as the Bullet World has been created, call the static method: ICollisionEvents::SetInstance(ICollisionEvents* instance,btDynamicsWorld* btWorld) like this:
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	// ICollisionEvents (MANDATORY)--------------
	ICollisionEvents::SetInstance(this,m_dynamicsWorld);	// "this" is good if this snippet is inside a method of the application class that inherits from ICollisionEvents
	// ----------------------------------------------
4) When you want to monitor a rigid body for collisions, create it as a btRigidBodyWithEvents instead of btRigidBody, and call:
    body->setMonitorCollisions(true);		
*/


/// TODO: Clean up and refactor this huge mess a bit, otherwise in a pair of weeks I won't be able to understand it anymore myself...

// Internal usage only (well, probably)
struct MyContactPoint
{
btCollisionObject* bodyB;
btVector3 localSpaceContactPoint;	
btVector3 worldSpaceContactPoint;
btVector3 worldSpaceContactNormal;	// from thisBodyA towards bodyB
btScalar penetrationDistance;
btScalar appliedImpulse;
btScalar numContactPoints;
bool startColliding;
MyContactPoint() 	{}
SIMD_FORCE_INLINE void set(btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse,const btScalar numContactPoints,const bool averageAllPoints)	{
	this->bodyB=bodyB;
	this->localSpaceContactPoint=localSpaceContactPoint;
	this->worldSpaceContactPoint=worldSpaceContactPoint;
	this->worldSpaceContactNormal=worldSpaceContactNormal;
	this->penetrationDistance=penetrationDistance;
	this->appliedImpulse=appliedImpulse;
	this->numContactPoints= (averageAllPoints ? numContactPoints : 1.0f);	
}
SIMD_FORCE_INLINE void add(const MyContactPoint& P,const bool averageAllPoints=false)	{
	btScalar newNumContactPoints=P.numContactPoints;
	if (!averageAllPoints)	{
		newNumContactPoints = numContactPoints + 1.0f;
	}
	btScalar newNumContactPointsInverse = (newNumContactPoints!=0 ? 1.0f/newNumContactPoints : 1.0f);
		
	localSpaceContactPoint=(localSpaceContactPoint * numContactPoints+ P.localSpaceContactPoint*P.numContactPoints)*newNumContactPointsInverse;
	worldSpaceContactPoint=(worldSpaceContactPoint * numContactPoints+ P.worldSpaceContactPoint*P.numContactPoints)*newNumContactPointsInverse;
	worldSpaceContactNormal=(worldSpaceContactNormal * numContactPoints+ P.worldSpaceContactNormal*P.numContactPoints)*newNumContactPointsInverse;
	penetrationDistance=(penetrationDistance * numContactPoints+ P.penetrationDistance*P.numContactPoints)*newNumContactPointsInverse;
	// The applied impulse must be averaged ?
	appliedImpulse=(appliedImpulse * numContactPoints+ P.appliedImpulse*P.numContactPoints)*newNumContactPointsInverse;
	// or just summed:
	//appliedImpulse+=P.appliedImpulse;	
	// P.S. This must be changed in btRigidBodyWithEvents::PerformCollisionDetection(...) as well
		
	numContactPoints =  newNumContactPoints;	
	//startColliding = P.startColliding;	// NO! THIS MUST BE LEFT OUT
}
SIMD_FORCE_INLINE static int FindRigidBodyIn(const btCollisionObject* body,const btAlignedObjectArray< MyContactPoint >& list)	{
	int size = list.size();
	for (int t = 0; t < size; t++ )	{
		if (body == list[t].bodyB) return t;
	}
	return -1;
}
static int AddAverageContactPointTo(const MyContactPoint& CP,btAlignedObjectArray< MyContactPoint >& list,const bool averageAllPoints=false)	{
	int index = FindRigidBodyIn(CP.bodyB,list);
	if (index==-1) {
		list.push_back(CP);
		return index;
	}
	else {
		list[index].add(CP,averageAllPoints);	
		return index;
	}
	return index;	
}
};

class btRigidBodyWithEvents;
// Inherit your application class from here:
class ICollisionEvents : public btActionInterface {
	public:
	virtual void  OnCollisionStart(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)=0;
	virtual void  OnCollisionContinue(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)=0;
	virtual void  OnCollisionStop(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)=0;	
		
	static void SetInstance(ICollisionEvents* instance,btDynamicsWorld* btWorld);

	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep);

	virtual void debugDraw(btIDebugDraw* debugDrawer) {}
	
	virtual ~ICollisionEvents() {}	
};


// This is just a partial part of btRigidBodyWithEvents (i.e. could just be merged to it):
///TODO: instead port everything we can from btRigidBodyWithEvents to this class, so that users
// may implement their own version of btRigidBodyWithEvents more easily (well, they can still inherit directly from
// it. Anyway the event handlers must take this class... it's not that easy at all for users to inglobe it)
class btRigidBodyWithEventsEventDelegates	{
protected:
	static ICollisionEvents* eventDispatcher;
	friend class ICollisionEvents;
	friend class btRigidBodyWithEvents;
	
	btAlignedObjectArray< MyContactPoint >* pLastCollidingBodies;	// A list of colliding bodies in the last frame
	btAlignedObjectArray< MyContactPoint >* pNewCollidingBodies;	// A list of colliding bodies in the this frame

	bool _monitorCollisions;
	//------------OnCollisionEvent-------------
	SIMD_FORCE_INLINE void  OnCollisionStart(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
		btAssert(eventDispatcher!=NULL);
		eventDispatcher->OnCollisionStart(thisBodyA,bodyB,localSpaceContactPoint,worldSpaceContactPoint,worldSpaceContactNormal,penetrationDistance,appliedImpulse);
	}
	SIMD_FORCE_INLINE void  OnCollisionContinue(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
		btAssert(eventDispatcher!=NULL);
		eventDispatcher->OnCollisionContinue(thisBodyA,bodyB,localSpaceContactPoint,worldSpaceContactPoint,worldSpaceContactNormal,penetrationDistance,appliedImpulse);
	}
	SIMD_FORCE_INLINE void  OnCollisionStop(btRigidBodyWithEvents* thisBodyA,btCollisionObject* bodyB,const btVector3& localSpaceContactPoint,const btVector3& worldSpaceContactPoint,const btVector3& worldSpaceContactNormal,const btScalar penetrationDistance,const btScalar appliedImpulse)	{
		btAssert(eventDispatcher!=NULL);
		eventDispatcher->OnCollisionStop(thisBodyA,bodyB,localSpaceContactPoint,worldSpaceContactPoint,worldSpaceContactNormal,penetrationDistance,appliedImpulse);
	}
	
	 //region  Completely Optional And Unused Stuff 
	SIMD_FORCE_INLINE MyContactPoint getContactPointForIndex(int index,bool* isOperationOKOut=NULL) const	{
		MyContactPoint point;
		btAlignedObjectArray< MyContactPoint >* pBodies = pLastCollidingBodies;	//pNewCollidingBodies is probably swapped and reset after each step.
		bool ok = true;
		if (pBodies && index>=0 && index<pBodies->size()) point = (*pBodies)[index];
		else ok = false;
		if (isOperationOKOut) *isOperationOKOut = ok;
		return point;
	}
	//endregion
	//------------------------------------------
public:
	 //region Ctr/Dtr
	btRigidBodyWithEventsEventDelegates() : _monitorCollisions(false), pLastCollidingBodies(NULL),pNewCollidingBodies(NULL)  {}
	~btRigidBodyWithEventsEventDelegates() {		
		if (pNewCollidingBodies)	{
			pNewCollidingBodies->clear();
			delete pNewCollidingBodies; pNewCollidingBodies = NULL;
		}
		if (pLastCollidingBodies)	{
			pLastCollidingBodies->clear();
			delete pLastCollidingBodies; pLastCollidingBodies = NULL;
		}
	}
	//endregion
	
	bool getMonitorCollisions() {return _monitorCollisions!=NULL;}
	
	 //region Completely Optional And Unused Stuff 
	bool isCollidingWith(const btCollisionObject* bodyB,int* collisionIndexOut=NULL)	{
		const btAlignedObjectArray< MyContactPoint >* pBodies = pLastCollidingBodies;	//pNewCollidingBodies is probably swapped and reset after each step.
		return (_monitorCollisions && pBodies && bodyB) ? 
				(
				!collisionIndexOut ? MyContactPoint::FindRigidBodyIn(bodyB,*pBodies)==-1 ? false : true :
				(*collisionIndexOut = MyContactPoint::FindRigidBodyIn(bodyB,*pBodies))==-1 ? false : true
				)
		 		: false;				
	}
	/** Similiar methods could be written for other variables
	    @param bodyB - the possible colliding body of the query. If 'optionalIndex' is known (e.g. from isCollidingWith) it is ignored.
	    @param contactPointLocalSpace - the return value
	    @param optionalIndex - if it is already known that the bodyB is colliding (through a previous 'instant' call to isCollidingWith(...) that
	                           can't be cached through different steps), reuse the "collisionIndexOut" of the previous call here.
		@return - true if contactPointLocalSpace has been filled
	*/
	bool getContactPointLocalSpace(const btCollisionObject* bodyB,btVector3& contactPointLocalSpace,int optionalIndex=-1)	{
		contactPointLocalSpace = btVector3(0.,0.,0.);
		if (optionalIndex!=-1) {
			bool ok;
			contactPointLocalSpace = getContactPointForIndex(optionalIndex,&ok).localSpaceContactPoint;
			return ok;
		}	
		bool ok = isCollidingWith(bodyB,&optionalIndex);
		if (!ok) return false;
		contactPointLocalSpace = getContactPointForIndex(optionalIndex).localSpaceContactPoint;
		return true;
	}
	btCollisionObject* isOnABody(const btScalar minYDifferenceBetweenBodyCOMAndCollisionPoint) const	{
		const btAlignedObjectArray< MyContactPoint >* pBodies = pLastCollidingBodies;	//pNewCollidingBodies is probably swapped and reset after each step.
		if (_monitorCollisions && pBodies)	{
			int size = (*pBodies).size();
			for (int t = 0; t < size; t++ )	{
				const MyContactPoint& CP = (*pBodies)[t];
				///TODO: Skip if CP.bodyB is camera
				if (CP.localSpaceContactPoint.y()<-minYDifferenceBetweenBodyCOMAndCollisionPoint) {
					return CP.bodyB;
				}
			}
		}
		return NULL;		
	}
	//endregion
};


class btRigidBodyWithEvents : public btRigidBody, public btRigidBodyWithEventsEventDelegates
{
	public:
	 //region Ctr/Dtr
	///btRigidBody constructor using construction info
	btRigidBodyWithEvents(	const btRigidBodyConstructionInfo& constructionInfo) : 	btRigidBody(constructionInfo)	{
		_init();
	}
	///btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	btRigidBodyWithEvents(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0)) 
		: 	btRigidBody(mass,motionState,collisionShape,localInertia) {
		_init();
	}
	virtual ~btRigidBodyWithEvents(){}
	//endregion	

	 //region upcast strategy
	#ifndef  USE_DYNAMIC_UPCAST
		//#define USE_DYNAMIC_UPCAST	// Twickable --- Pro: user ptr is left free; Cons: probably slower 
	#endif //USE_DYNAMIC_UPCAST
	static const btRigidBodyWithEvents*	upcast(const btCollisionObject* colObj)	{
		//if (colObj->getInternalType()==btCollisionObject::CO_RIGID_BODY)	// Mmmh, this won't allow upcasting both btRigidBody::upcast and btRigidBodyWithEvents::upcast work correctly with a btRigidBody* that it's not a btRigidBodyWithEvents*.
		#ifdef USE_DYNAMIC_UPCAST
		return dynamic_cast < const btRigidBodyWithEvents* > (colObj);
		#else //USE_DYNAMIC_UPCAST
		if ((int)(colObj->getUserPointer())==upcastUserPointerForThisClass) return (const btRigidBodyWithEvents*)colObj;
		return 0;
		#endif //USE_DYNAMIC_UPCAST
	}
	static btRigidBodyWithEvents*	upcast(btCollisionObject* colObj)	{
		#ifdef USE_DYNAMIC_UPCAST
		return dynamic_cast < btRigidBodyWithEvents* > (colObj);
		#else//USE_DYNAMIC_UPCAST
		if ((int)(colObj->getUserPointer())==upcastUserPointerForThisClass) return (btRigidBodyWithEvents*)colObj;
		return 0;
		#endif//USE_DYNAMIC_UPCAST
	}
	//endregion

	void setMonitorCollisions(bool flag=true);

	protected:
	
	friend class ICollisionEvents;
	static const int upcastUserPointerForThisClass = 1;	// Used only if USE_DYNAMIC_UPCAST is NOT defined
	void _init()	{
		#ifndef USE_DYNAMIC_UPCAST
			this->setUserPointer((void*)upcastUserPointerForThisClass);
		#else//USE_DYNAMIC_UPCAST
			#undef USE_DYNAMIC_UPCAST	//no more needed
		#endif//USE_DYNAMIC_UPCAST	
	}
	
	static void PerformCollisionDetection(btDynamicsWorld* btWorld,bool processContactPointWithPositiveDistancesToo=false,bool averageAllContactPointsInDifferentManifolds=false);
	static void _SendCollisionEvents();
	static btAlignedObjectArray< btRigidBodyWithEvents* > monitorCollisionsBodies;	// This list is the list of bodies for which to monitor collisions
	SIMD_FORCE_INLINE static int FindRigidBodyIn(const btRigidBodyWithEvents* body,const btAlignedObjectArray< btRigidBodyWithEvents* >& list)	{
		int size = list.size();
		for (int t = 0; t < size; t++ )	{
			if (body == list[t]) return t;
		}
		return -1;
	}

};


#endif //BTRIGIDBODYWITHCOLLISIONEVENTS_H__



