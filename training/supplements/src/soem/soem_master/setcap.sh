#!/bin/bash

sudo setcap cap_net_raw+ep `which deployer-$OROCOS_TARGET`
if [ $? -ne 0 ]; then
    echo "Failed to give deployer-$OROCOS_TARGET raw network capabilities"
    exit $?
fi
sudo setcap cap_net_raw+ep `which rttlua-$OROCOS_TARGET`
if [ $? -ne 0 ]; then
    echo "Failed to give rttlua-$OROCOS_TARGET raw network capabilities"
    exit $?
fi
echo "Successfully gave deployer-$OROCOS_TARGET and rttlua-$OROCOS_TARGET raw network capabilities"

exit 0
