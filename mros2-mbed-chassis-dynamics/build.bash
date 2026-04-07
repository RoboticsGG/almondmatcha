#!/bin/bash

### setup operation ###
if [ $# -ne 1 -a $# -ne 3 -a $# -ne 4 ];
then
  echo "ERROR: args invalid"
  echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>] [{native|docker}]"
  exit 1
fi

if [ ${1} = "all" ];
then
  echo "INFO: operation is make all"
  MAKECMD="all"
elif [ ${1} = "rebuild" ];
then
  echo "INFO: operation is rebuild"
  MAKECMD="rebuild"
elif [ ${1} = "clean" ];
then
  echo "INFO: operation is clean"
  MAKECMD="clean"
elif [ ${1} = "distclean" ];
then
  echo "INFO: distclean operation will be done by root privilege"
  sudo rm -rf build/ mros2/ mbed-os/
  echo "INFO: distclean completed"
  exit 0
else
  echo "ERROR: args invalid"
  echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>] [{native|docker}]"
  exit 1
fi

if [ $# -eq 1 ];
then
  TARGET="NUCLEO_F767ZI"
  APPNAME="echoback_string"
  echo "WARN: default set will be used for build"
else
  TARGET=$2
  APPNAME=$3
fi

DOCKERCMD_PRE="docker run --rm -it --mount type=bind,source=$(pwd),destination=/var/mbed \
  -w /var/mbed -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c \"git config --global --add safe.directory '*' && "
DOCKERCMD_SUF="\""
if [ $# == 4 ];
then
  if [ ${4} = "native" ];
  then
    echo "INFO: build operation will be executed on native env"
    DOCKERCMD_PRE=""
    DOCKERCMD_SUF=""
    export APPNAME=${APPNAME}
  elif [ ${4} = "docker" ];
  then
    echo "INFO: build operation will be executed on dokcer env"
  else
    echo "ERROR: args invalid"
    echo "ERROR: \"native\" or \"docker\" should be specified as 4th arg"
    echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>]"
    exit 1
  fi
else
  echo "INFO: build operation will be executed on dokcer env"
fi

echo "INFO: build setting"
echo "      TARGET=${TARGET}"
echo "      APPNAME=${APPNAME}"


### build a project ###
# mbed-tools deploy cannot be used here because the vendored sensor libs in libs/
# exist as plain source directories (not git repos), which causes deploy to error with
# "Could not find a valid git repository at this path".  Instead, clone mbed-os and
# mros2 directly from the pinned refs in the .lib files.  After cloning, sync our
# custom msgs_ifaces headers into mros2/mros2_msgs/ (mros2 from GitHub only has std msgs).
DEPLOY_SYNC_CMD='([ -d mbed-os ] || (git clone https://github.com/ARMmbed/mbed-os.git mbed-os && git -C mbed-os checkout d723bf9e55415433e108124ee6d36337feddf1b8)) && ([ -d mros2 ] || git clone --branch v0.5.4 https://github.com/mROS-base/mros2.git mros2) && if [ -d mros2/mros2_msgs ] && [ -d mros2_add_msgs/mros2_msgs/msgs_ifaces/msg ]; then mkdir -p mros2/mros2_msgs/msgs_ifaces/msg && cp -f mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/*.hpp mros2/mros2_msgs/msgs_ifaces/msg/ && echo INFO: synced msgs_ifaces headers into mros2/mros2_msgs/; fi'
eval ${DOCKERCMD_PRE}${DEPLOY_SYNC_CMD}${DOCKERCMD_SUF}

# configure mbed project by mbed-tools (output to build directory)
eval ${DOCKERCMD_PRE}mbed-tools configure -m ${TARGET} -t GCC_ARM -o build${DOCKERCMD_SUF}

# set the build parameter
eval ${DOCKERCMD_PRE}cmake -S . -B build -GNinja${DOCKERCMD_SUF}

# cmake calls templates_generator.py which has a toSnakeCase bug: ChassisIMU -> chassis_im_u
# (consecutive uppercase letters split incorrectly).  Fix the generated file before compiling.
eval ${DOCKERCMD_PRE}sed -i 's/chassis_im_u\\.hpp/chassis_imu.hpp/g' platform/templates.hpp${DOCKERCMD_SUF}

# build (switch according to arg1)
if [ ${MAKECMD} = "all" ];
then
  eval ${DOCKERCMD_PRE}cmake --build build${DOCKERCMD_SUF}
elif [ ${MAKECMD} = "rebuild" ];
then
  eval ${DOCKERCMD_PRE}cmake --build build --clean-first${DOCKERCMD_SUF}
elif [ ${MAKECMD} = "clean" ];
then
  eval ${DOCKERCMD_PRE}cmake --build build --target clean${DOCKERCMD_SUF}
fi

# Docker runs as root → all generated files (mbed-os/, mros2/, build/) are owned by
# root.  Fix ownership back to the invoking user so the next run works without sudo.
if [ -n "${SUDO_USER:-}" ]; then
  chown -R "${SUDO_USER}":"${SUDO_USER}" build/ mros2/ mbed-os/ platform/ 2>/dev/null || true
fi
