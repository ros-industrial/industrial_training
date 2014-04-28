all: installed

TARBALL     = build/ARToolKit-SVN-r311.tar.gz
TARBALL_URL = https://code.ros.org/svn/release/download/thirdparty/ARToolKit-SVN-r311.tar.gz
UNPACK_CMD  = tar zxf
SOURCE_DIR  = build/artoolkit
TARBALL_PATCH = patch_auto_config patch_v4l
MD5SUM_FILE = ARToolKit-SVN-r311.tar.gz.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	#Build Artoolkit with GStreamer change line below with: cd $(SOURCE_DIR) && ./Configure 5 y y y 
	#Currently build with V4L
	cd $(SOURCE_DIR) && ./Configure 1 n n n
	cd $(SOURCE_DIR) && make
	#remove .svn folders from SOURCE_DIR
	cd $(SOURCE_DIR) && rm -rf `find . -type d -name .svn`
	mkdir -p $(CURDIR)/lib
	cp -r $(CURDIR)/$(SOURCE_DIR)/lib/*.a $(CURDIR)/lib/
	mkdir -p $(CURDIR)/include
	cp -r $(CURDIR)/$(SOURCE_DIR)/include/* $(CURDIR)/include/
	touch installed

clean:
	cd $(SOURCE_DIR) && make clean
	rm -rf $(CURDIR)/lib $(CURDIR)/include stage installed patched 

wipe: clean
	rm -rf $(SOURCE_DIR)
	rm -rf build
