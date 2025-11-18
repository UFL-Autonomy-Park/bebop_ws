
# Build the arsdk3, 
set(built_libs ${REPODIR}/out/arsdk-native/staging/usr/lib)
set(built_includes ${REPODIR}/out/arsdk-native/staging/usr/include/)

add_custom_command(
	OUTPUT ${built_libs} ${built_includes}
	COMMAND ./build.sh -p arsdk-native -t build-sdk -j
	WORKING_DIRECTORY ${REPODIR}
	COMMENT "Building parrot ARSDK3"
	)

add_custom_target(arsdk3_build ALL DEPENDS ${built_libs})
add_dependencies(arsdk3_build arsdk3_download)
