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
# Deploy libraries: clone mbed-os and mros2 directly from the refs in .lib files.
# mbed-tools deploy fails on a clean directory (needs existing git repos to work).
# Direct clone is idempotent — skips if directory already exists.
DEPLOY_CMD='([ -d mbed-os ] || (git clone https://github.com/ARMmbed/mbed-os.git mbed-os && git -C mbed-os checkout d723bf9e55415433e108124ee6d36337feddf1b8)) && ([ -d mros2 ] || git clone --branch v0.5.4 https://github.com/mROS-base/mros2.git mros2) && git -C mros2 submodule update --init'
eval ${DOCKERCMD_PRE}${DEPLOY_CMD}${DOCKERCMD_SUF}

# Apply embeddedRTPS patches (e.g. AckNack bitmap overflow fix)
if [ -d platform/patches ] && [ -d mros2/embeddedRTPS ]; then
  for p in platform/patches/*.patch; do
    [ -f "$p" ] || continue
    abspath="$(cd "$(dirname "$p")" && pwd)/$(basename "$p")"
    if ! git -C mros2/embeddedRTPS apply --check --reverse "$abspath" 2>/dev/null; then
      git -C mros2/embeddedRTPS apply "$abspath" && echo "INFO: applied patch $(basename $p)"
    fi
  done
fi

# configure mbed project by mbed-tools (output to build directory)
eval ${DOCKERCMD_PRE}mbed-tools configure -m ${TARGET} -t GCC_ARM -o build${DOCKERCMD_SUF}

# set the build parameter
eval ${DOCKERCMD_PRE}cmake -S . -B build -GNinja${DOCKERCMD_SUF}

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

# Docker runs as root -> all generated files (mbed-os/, mros2/, build/) are owned by
# root.  Fix ownership back to the invoking user so the next run works without sudo.
if [ -n "${SUDO_USER:-}" ]; then
  chown -R "${SUDO_USER}":"${SUDO_USER}" build/ mros2/ mbed-os/ platform/ 2>/dev/null || true
fi
