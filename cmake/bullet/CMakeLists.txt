cmake_minimum_required ( VERSION 2.8.11 )

project ( bullet )
set ( BULLET_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../bullet/src" )
set ( BULLET_INCLUDES "${BULLET_SOURCE_DIR}" )    

file ( GLOB_RECURSE BULLET_SOURCES "${BULLET_SOURCE_DIR}/*.cpp"
                    "${BULLET_SOURCE_DIR}/BulletDynamics/*.cpp"
                    "${BULLET_SOURCE_DIR}/BulletCollision/*.cpp"
                    "${BULLET_SOURCE_DIR}/BulletSoftBody/*.cpp"
                    "${BULLET_SOURCE_DIR}/LinearMath/*.cpp"
)
file ( GLOB_RECURSE BULLET_HEADERS  "${BULLET_SOURCE_DIR}/*.h"
                                    "${BULLET_SOURCE_DIR}/BulletDynamics/*.h"
                                    "${BULLET_SOURCE_DIR}/BulletCollision/*.h"
                                    "${BULLET_SOURCE_DIR}/BulletSoftBody/*.h"
                                    "${BULLET_SOURCE_DIR}/LinearMath/*.h"
)


add_library( bullet STATIC ${BULLET_SOURCES} ${BULLET_HEADERS} "${BULLET_SOURCE_DIR}/../.." ) 
target_include_directories ( bullet PUBLIC ${BULLET_INCLUDES})
set_target_properties( bullet PROPERTIES FOLDER Third-Party )



