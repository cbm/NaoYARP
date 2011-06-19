
export CTC_DIR=$(readlink -f "$1")




export BUILD_ARG="$@"

export PATH_CC="${CTC_DIR}/cross/geode/bin"

export CXX=${PATH_CC}/i586-linux-g++
export CC=${PATH_CC}/i586-linux-gcc
export LD=${PATH_CC}/i586-linux-ld
export AR=${PATH_CC}/i586-linux-ar
export RANLIB=${PATH_CC}/i586-linux-ranlib

export CPPFLAGS="--sysroot ${CTC_DIR}/staging/geode-linux/ \
-I${CTC_DIR}/staging/geode-linux/usr/include/ \
-I${CTC_DIR}/cross/geode/i586-linux/include/c++/ \
-I${CTC_DIR}/cross/geode/i586-linux/include/c++/i586-linux/ \
-I${CTC_DIR}/cross/geode/i686-linux/i586-linux/include \
-I${CTC_DIR}/cross/geode/lib/gcc/i586-linux/4.3.3/include "

export CFLAGS="--sysroot ${CTC_DIR}/staging/geode-linux/ \
-I${CTC_DIR}/staging/geode-linux/usr/include/ \
-I${CTC_DIR}/cross/geode/i586-linux/include/c++/ \
-I${CTC_DIR}/cross/geode/i686-linux/i586-linux/include \
-I${CTC_DIR}/cross/geode/lib/gcc/i586-linux/4.3.3/include "

export LDFLAGS="--sysroot ${CTC_DIR}/staging/geode-linux/ \
-lgcc -L${CTC_DIR}/cross/geode/lib/ -lc -lstdc++ -ldl"

