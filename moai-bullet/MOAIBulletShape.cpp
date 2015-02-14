// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com


//NEED TO CHANGE THE SHAPES TO THE BASIC
// http://bulletphysics.com/Bullet/BulletFull/classbtSphereShape.html


#include "pch.h"
#include <moai-bullet/MOAIBulletShape.h>
#include <bullet/src/btBulletDynamicsCommon.h>

//----------------------------------------------------------------//
int MOAIBulletShape::_makePlane ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	float width  = state.GetValue < float >( 2, 0.0f );	
	float height = state.GetValue < float >( 3, 0.0f ); 
	float lenght = state.GetValue < float >( 4, 0.0f ); 	
	self->mShape = new btStaticPlaneShape(btVector3(width, height, lenght),0);

	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 10, 0), 0);
	//btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, self->mMotion, self->mShape, btVector3(0, 0, 0));
	//self->mBody = new btRigidBody(groundRigidBodyCI);
return 1;

};
//----------------------------------------------------------------//
int MOAIBulletShape::_makeSphere ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
		float radius = state.GetValue < float >( 2, 0.0f );
		self->mShape = new btSphereShape(radius);	
		//btSphereShape * mShape = new btCylinderShape(btVector3(width, height, lenght)); 
		//btRigidBody::btRigidBodyConstructionInfo mCI(self->mMass,  self->mMotion, self->mShape, self->mInertia);
		//self->mBody = new btRigidBody(mCI);  
	return 1;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_makeBox ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	
	float width  = state.GetValue < float >( 2, 0.0f );	
	float height = state.GetValue < float >( 3, 0.0f ); 
	float lenght = state.GetValue < float >( 4, 0.0f ); 

	self->mShape = new btBoxShape(btVector3(width, height, lenght)); 

	//self->mShape->setMargin( 0.001 );
	//btBoxShape * mShape = new btCylinderShape(btVector3(width, height, lenght));
	//self->mShape->calculateLocalInertia(self->mMass, self->mInertia);
	//btRigidBody::btRigidBodyConstructionInfo mCI(self->mMass, self->mMotion, self->mShape, self->mInertia);
	//self->mBody = new btRigidBody(mCI);    


	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_makeCylinder ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )

	float width  = state.GetValue < float >( 2, 0.0f );	
	float height = state.GetValue < float >( 3, 0.0f ); 
	float lenght = state.GetValue < float >( 4, 0.0f ); 
	self->mShape = new btCylinderShape(btVector3(width, height, lenght)); 
	//btCylinderShape * mShape = new btCylinderShape(btVector3(width, height, lenght)); 
	//btRigidBody::btRigidBodyConstructionInfo mCI(self->mMass, self->mMotion, self->mShape, self->mInertia);
	//self->mBody = new btRigidBody(mCI);    
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_makeCapsule ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )

	float radius  = state.GetValue < float >( 2, 0.0f );	
	float height = state.GetValue < float >( 3, 0.0f ); 	
	self->mShape  = new btCapsuleShape(radius, height);

	//BODY
	//btCapsuleShape * mShape = new btCapsuleShape(radius, height); 
	//btRigidBody::btRigidBodyConstructionInfo mCI(self->mMass, self->mMotion, self->mShape, self->mInertia);
	//self->mBody = new btRigidBody(mCI);   

	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_makeCone ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	float radius  = state.GetValue < float >( 2, 0.0f );	
	float height = state.GetValue < float >( 3, 0.0f ); 

	self->mShape = new btConeShape (radius, height);

	//btConeShape  * mShape = new btConeShape (radius, height); 
	//btRigidBody::btRigidBodyConstructionInfo mCI(self->mMass, self->mMotion, self->mShape, self->mInertia);
	//self->mBody = new btRigidBody(mCI);  

	return 0;



};
//----------------------------------------------------------------//
int MOAIBulletShape::_SetPosition ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	self->mLoc_x		= state.GetValue < float >( 2, 0.0f );	
	self->mLoc_y		= state.GetValue < float >( 3, 0.0f ); 
	self->mLoc_z		= state.GetValue < float >( 4, 0.0f ); 
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_SetRotation ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	self->mRot_x	= state.GetValue < float >( 2, 0.0f );	
	self->mRot_y	= state.GetValue < float >( 3, 0.0f ); 
	self->mRot_z	= state.GetValue < float >( 4, 0.0f ); 
	return 0;

};
//----------------------------------------------------------------//
int MOAIBulletShape::_setMargin ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	float margin= state.GetValue < float >( 2, 0.0f );	
	self->mShape->setMargin(margin);
	return 0;
};
//----------------------------------------------------------------//
int MOAIBulletShape::_setMass ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIBulletShape, "UNN" );	

	float mass	  = state.GetValue < float >( 2, 0.0f );
	float Inertia = state.GetValue < float >( 3, 0.0f );

	//MASS
		btScalar mMass = btScalar(mass);
	//inertia
		btVector3 mInertia(Inertia, Inertia, Inertia);	
	//APPLY
		self->mShape->calculateLocalInertia(mMass, mInertia);

	//UPDATE // THIS IS WRONG BECAUSE IT CACLUATES MASS FROM SHAPES
		self->mBody->setMassProps(mMass, mInertia);
   
	return 0;
}

//----------------------------------------------------------------//
int MOAIBulletShape::_addToBody ( lua_State* L ) {
MOAI_LUA_SETUP ( MOAIBulletShape, "U" )
	btTransform t; 
	t.setIdentity();
	t.setRotation ( btQuaternion ( self->mRot_x, self->mRot_y,self->mRot_z)); 
	t.setOrigin(btVector3 ( self->mLoc_x, self->mLoc_y,self->mLoc_z));
	self->mCompound->addChildShape(t,self->mShape); //MAKE FRIEND CLASS
return 0;
};
//----------------------------------------------------------------//
void MOAIBulletShape::setCompound (btCompoundShape*	mCompound) {
	this->mCompound = mCompound;
};
//----------------------------------------------------------------//
//REMBERING THE BODY??
void MOAIBulletShape::setBody (btRigidBody*		mBody) {
	this->mBody = mBody;
};
//----------------------------------------------------------------//
void MOAIBulletShape::setOrigin	(float loc_x,float loc_y,float loc_z) {
	this->mLoc_x = loc_x;
	this->mLoc_y = loc_y;
	this->mLoc_z = loc_z;	
};
//----------------------------------------------------------------//
void MOAIBulletShape::setEulerZYX	(float rot_x,float rot_y,float rot_z) {
	this->mRot_x = rot_x;
	this->mRot_y = rot_y;
	this->mRot_z = rot_z;
};
//----------------------------------------------------------------//
void MOAIBulletShape::Destroy () {
}
//----------------------------------------------------------------//
MOAIBulletShape::MOAIBulletShape () :
mRot(0),
mLoc(0),
mShape(0),
mLoc_x(0),
mLoc_y(0),
mLoc_z(0),
mRot_x(0),
mRot_y(0),
mRot_z(0)

{	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAILuaObject )
	RTTI_END	
};
//----------------------------------------------------------------//
MOAIBulletShape::~MOAIBulletShape () {
	printf(" \n ~MOAIBulletShape \n");
	this->Destroy ();
}
//----------------------------------------------------------------//
void MOAIBulletShape::RegisterLuaClass ( MOAILuaState& state ) {
	
}
//----------------------------------------------------------------//
void MOAIBulletShape::RegisterLuaFuncs ( MOAILuaState& state ) {

luaL_Reg regTable [] = {

	{ "makeSphere",		_makeSphere },
	{ "makeBox",		_makeBox },	
	{ "makeCylinder",	_makeCylinder },	
	{ "makeCapsule",	_makeCapsule },		
	{ "makeCone",		_makeCone },		
	{ "makePlane",		_makePlane },

	{ "setMass",		_setMass },
	{ "setMargin",		_setMargin },

	{ "addToBody",		_addToBody },	

	{ "setPosition",		_SetPosition },		
	{ "setRotation",		_SetRotation },



{ NULL, NULL }
};
	
luaL_register ( state, 0, regTable );
}



