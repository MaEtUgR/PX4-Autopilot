if(${OS} STREQUAL "nuttx")
	list(APPEND MODULE_CFLAGS -Wframe-larger-than=2048)
elseif(${OS} STREQUAL "posix")
	list(APPEND MODULE_CFLAGS -Wno-error)
endif()

# use custom matrix lib instead of Eigen
add_definitions(-DUSE_MATRIX_LIB)

px4_add_module(
	MODULE modules__mcontrol
	MAIN mcontrol
	STACK 2048
	COMPILE_FLAGS ${MODULE_CFLAGS}
	SRCS
		mcontrol_main.cpp
		BlockMControl.cpp
		params.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
