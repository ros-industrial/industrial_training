TARBALL = SOEM1.2.5.tar.bz2
TARBALL_URL = http://download.berlios.de/soem/SOEM1.2.5.tar.bz2
SOURCE_DIR = build/soem_core
INITIAL_DIR = build/SOEM1.2.5
UNPACK_CMD = tar xjf
MD5SUM_FILE = SOEM1.2.5.tar.bz2.md5sum
TARBALL_PATCH=rtnet.patch

all: $(SOURCE_DIR)/unpacked

include $(shell rospack find mk)/download_unpack_build.mk

unpack: $(SOURCE_DIR)

wipe: clean
	-rm -rf $(SOURCE_DIR)
	-rm -rf ${TARBALL}

EXTRA_CMAKE_FLAGS=-DENABLE_RTNET=OFF
include $(shell rospack find mk)/cmake.mk