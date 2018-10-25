#!/usr/bin/env bash

# This program downloads the selected SDK from Nordic's official
# repository and update it with the BLEnd patch for continuous
# neighbor discovery.

# Template projects will be created in the SDK's project folder.

declare -a SDK_ADDR=("https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip" \
		     "https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v14.x.x/nRF5_SDK_14.2.0_17b948a.zip" \
		     "https://github.com/NordicSemiconductor/Nordic-Thingy52-FW/archive/v2.1.0.zip"
		    )
declare -a SDK_NAMES=("nRF5_SDK_v15" "nRF5_SDK_v14" "Thingy_IoT_SensorKit_v2.1.0")

ARCHIVE_NAME="@ARCHIVE_NAME@"
ROOT_DIR=$(pwd)
SDK_DIR="${ROOT_DIR}/sdk"
SDK_PATH="@SDK_PATH"
SDK_VERSION="@SDK_VERSION@"
VERSION_LENGTH="${#SDK_ADDR[@]}"

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
	echo "[$i]:${SDK_NAMES[$i]}"
    done

    read -p "Enter the SDK# and press [ENTER]: " in_sdk_version

    if [ "$in_sdk_version" -ge 0 -a "$in_sdk_version" -le 2 ];
    then SDK_VERSION=$in_sdk_version;
    else echo "Invalid version selected."      
	 exit 1;
    fi
}

download_sdk()
{
    URL="${SDK_ADDR[$SDK_VERSION]}"
    mkdir -p "${SDK_DIR}"
    ARCHIVE_NAME="${URL##*/}"
    echo "Downloading SDK [${SDK_NAMES[$SDK_VERSION]}]"
    if [ -f "${SDK_DIR}/${ARCHIVE_NAME}" ]; then
    	echo "${ARCHIVE_NAME} exists, skip downloading."
    else
    	echo "Downloading SDK (${URL}) to ${ROOT_DIR}/${SDK_DIR}"
    	wget --directory-prefix "${SDK_DIR}" --progress=bar "${URL}"
    fi

    echo "Extracting SDK [${SDK_NAMES[$SDK_VERSION]}]"
    SDK_PATH="${SDK_DIR}/${ARCHIVE_NAME%.*}"
    if [ ! -d "${SDK_PATH}" ]; then
	unzip -o "${SDK_DIR}/$ARCHIVE_NAME" -d "${SDK_DIR}" | pv -l >/dev/null
    else
	echo "Directory exists, skip extracting."
    fi
}

show_welcome

version_select

download_sdk
