
#include "pch.h"
#include <moai-bullet/MOAIBulletDebugDraw.h>

//void  MOAIBulletDebugDraw::drawSphere(btScalar radius, const btTransform& transform, const btVector3& color){//
//	printf("drawSphere B \n");//
//};
//----------------------------------------------------------------//
//void  MOAIBulletDebugDraw::drawSphere (const btVector3& p, btScalar radius, const btVector3& color){
//		printf("drawSphere A \n");
//};
//----------------------------------------------------------------//
//void  MOAIBulletDebugDraw::drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& /*n0*/,const btVector3& /*n1*/,const btVector3& /*n2*/,const btVector3& color, btScalar alpha){
//		printf("drawTriangle \n");
//};
//----------------------------------------------------------------//
//void  MOAIBulletDebugDraw::drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar /*alpha*/){
//	//printf("drawTriangle \n");
//};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color){
//	//printf("drawBox \n");
//};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawBox(const btVector3& bbMin, const btVector3& bbMax, const btTransform& trans, const btVector3& color){
//	//printf("drawBox \n");
//};
//
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color){
//	//printf("drawCapsule \n");
//};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color){
//	//printf("drawCylinder \n");
//};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawCone(btScalar radius, btScalar height, int upAxis, const btTransform& transform, const btVector3& color){
//	//printf("drawCone \n");
//};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawPlane(const btVector3& planeNormal, btScalar planeConst, const btTransform& transform, const btVector3& color){
//	printf("drawPlane \n");
//};
//void MOAIBulletDebugDraw::drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, const btVector3& color, bool drawSect, btScalar stepDegrees = btScalar(10.f)); //----------------//
//void MOAIBulletDebugDraw::drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis, btScalar radius, btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3& color, btScalar stepDegrees = btScalar(10.f),bool drawCenter = true);

//----------------------------------------------------------------//
void  MOAIBulletDebugDraw::drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color){
	//printf("drawContactPoint \n");
};
//----------------------------------------------------------------//
bool    MOAIBulletDebugDraw::isVisible(const btVector3& aabbMin,const btVector3& aabbMax){
	//printf("isVisible \n");
	return true;
};
//----------------------------------------------------------------//
void  MOAIBulletDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3& color){
MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get ();		
gfxDevice.SetPenColor ( color.x(), color.y(), color.z(), 1.0f );
mSize = mSize + (1);
//FROM
ZLVec3D vtx_from;
vtx_from.mX = from.x() * this->mScale;
vtx_from.mY = from.y() * this->mScale;
vtx_from.mZ = from.z() * this->mScale;
gfxDevice.WriteVtx ( vtx_from );
gfxDevice.WriteFinalColor4b ();
//TO
ZLVec3D vtx_to;
vtx_to.mX = to.x() * this->mScale;
vtx_to.mY = to.y() * this->mScale;
vtx_to.mZ = to.z() * this->mScale;
gfxDevice.WriteVtx ( vtx_to );
gfxDevice.WriteFinalColor4b ();		

};
//----------------------------------------------------------------//
void    MOAIBulletDebugDraw::drawLine(const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor){
	printf("drawLineB \n");
};
//----------------------------------------------------------------//
void  MOAIBulletDebugDraw::reportErrorWarning(const char* warningString){
	printf("reportErrorWarning \n");
};
//----------------------------------------------------------------//
void  MOAIBulletDebugDraw::draw3dText(const btVector3& location,const char* textString){
	printf("draw3dText \n");
};
//----------------------------------------------------------------//
void  MOAIBulletDebugDraw::setDebugMode(int debugMode){
	printf("setDebugMode \n");
}; 
//----------------------------------------------------------------//
int    MOAIBulletDebugDraw::getDebugMode()const{
	//return (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits);

		return (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits);
	//return (btIDebugDraw::DBG_FastWireframe | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits);
	

};
//----------------------------------------------------------------//
void MOAIBulletDebugDraw::drawAabb(const btVector3& from,const btVector3& to,const btVector3& color){
	printf("drawAabb \n");

};
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::drawTransform(const btTransform& transform, btScalar orthoLen){
//	printf("drawTransform \n");
//};
//----------------------------------------------------------------//
void MOAIBulletDebugDraw::WriteVtx ( MOAIGfxDevice& gfxDevice, float x, float y,float z ) {
ZLVec3D vtx;
vtx.mX = x * this->mScale;
vtx.mY = y * this->mScale;
vtx.mZ = z * this->mScale;
gfxDevice.WriteVtx ( vtx );
gfxDevice.WriteFinalColor4b ();
}
//----------------------------------------------------------------//
//void MOAIBulletDebugDraw::resetSize ( ) {
//	mSize = 0;
//}




