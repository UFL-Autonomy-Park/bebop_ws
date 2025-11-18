
set(ARSDK_BUILD_ROOTDIR ${REPODIR}/out/arsdk-native/staging)
install(DIRECTORY ${ARSDK_BUILD_ROOTDIR}/usr/include/ DESTINATION include/ )
install(DIRECTORY ${ARSDK_BUILD_ROOTDIR}/usr/lib/ DESTINATION lib/ )
install(DIRECTORY ${ARSDK_BUILD_ROOTDIR}/usr/share/ DESTINATION share/ )
install(DIRECTORY ${ARSDK_BUILD_ROOTDIR}/etc/ DESTINATION etc/ )
