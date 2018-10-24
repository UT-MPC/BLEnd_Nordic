#!/usr/bin/env bash

# This program downloads the selected SDK from Nordic's official
# repository and update it with the BLEnd patch for continuous
# neighbor discovery.

# Template projects will be created in the SDK's project folder.

SDK_VERSION="@SDK_VERSION@"
declare -a SUPPORT_VERSION_NAMES=("nRF5_SDK_v15" "nRF5_SDK_v14" "Thingy_IoT_SensorKit_v2.1.0")
VERSION_LENGTH=${#SUPPORT_VERSION_NAMES[@]}

show_welcome()
{
    echo -e "This script helps you to download the Nordic SDK you\n" \
    "need and update it with the BLEnd patch for continuous\n" \
    "neighbor discovery.\n"
}

version_select()
{
    echo -e "- Which SDK version do you need?"
    for (( i=0; i<${VERSION_LENGTH};i++ ));
    do
	echo "[$i]:${SUPPORT_VERSION_NAMES[$i]}"
    done

    read -p "Enter the SDK# and press [ENTER]: " in_sdk_version

    if [ "$in_sdk_version" -ge 1 -a "$in_sdk_version" -le 3 ];
    then SDK_VERSION=$in_sdk_version;
    else echo "Invalid version selected."      
	 exit 1;
    fi
}

show_welcome

version_select
echo "${SDK_VERSION}"
