include(FetchContent)

# Get the repo tool 
FetchContent_Declare(
  repo
  URL https://storage.googleapis.com/git-repo-downloads/repo
  DOWNLOAD_NO_EXTRACT TRUE
  SOURCE_DIR ${REPODIR}
)

FetchContent_MakeAvailable(
	repo )

file(CHMOD ${REPODIR}/repo PERMISSIONS OWNER_EXECUTE OWNER_READ)

add_custom_command(
	OUTPUT ${REPODIR}/packages/ARSDK3
	COMMAND ${REPODIR}/repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -b ${ARSDK_MANIFEST_HASH} -m release.xml 
	COMMAND ${REPODIR}/repo sync -c
	WORKING_DIRECTORY ${REPODIR}
	COMMENT "Downloading parrot ARSDK3 with repo in ${REPODIR}"
    COMMENT "Now patching pymavlink for fixing python 3.9 error"
	COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/patches/mavcrc.py.patch
	)
add_custom_target(arsdk3_download ALL DEPENDS ${REPODIR}/packages/ARSDK3)

