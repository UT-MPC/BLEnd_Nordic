#!/usr/bin/env bash

# This program downloads the selected SDK from Nordic's official
# repository and update it with the BLEnd patch for continuous
# neighbor discovery.

# Template projects will be created in the SDK's project folder.

declare -a PREREQUISITES=("pv" "wget")
declare -a SDK_ADDR=("https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip" \
		     "https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v14.x.x/nRF5_SDK_14.2.0_17b948a.zip" \
		     "https://github.com/NordicSemiconductor/Nordic-Thingy52-FW/archive/v2.1.0.zip"
		    )
declare -a SDK_NAMES=("nRF5_SDK_v15" "nRF5_SDK_v14" "Thingy_IoT_SensorKit_v2.1.0")
declare -a THINGY_SDK_CHANGES=("include" "source")

ARCHIVE_NAME="@ARCHIVE_NAME@"
BLEND_SRC_DIR="src/blend"
BLEND_TEMP_ORI_LOCATION="src/blend_project_templates"
BLEND_TEMP_NAME="blend_app_template"
COMPILE_THINGY_SDK_URL="https://github.com/NordicSemiconductor/Nordic-Thingy52-FW/blob/master/README.md"
COMPILE_THINGY_URL="https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation/firmware_compile.html"
COMPLIE_NRF52_URL="https://devzone.nordicsemi.com/tutorials/b/getting-started/posts/development-with-gcc-and-eclipse"
NRF_TOOL_INSTALLED=false
NRF_TOOL_NAME="nrfjprog"
ROOT_DIR=$(pwd)
SDK_DIR="${ROOT_DIR}/sdk"
SDK_PATH="@SDK_PATH"
SDK_VERSION="@SDK_VERSION@"
THINGY_ARCHIVE_NAME="Nordic-Thingy52-FW-2.1.0"
THINGY_SDK_MOD_DIR="src/thingy_sdk_mod"
TOOLCHAIN_URL="http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.tools/dita/tools/nrf5x_command_line_tools/nrf5x_installation.html"
VERSION_LENGTH="${#SDK_ADDR[@]}"

check_prerequisite(){
    echo -e "Checking prerequisites."
    for (( i=0; i<${#PREREQUISITES[@]};i++));
    do
	if ! [ -x "$(command -v ${PREREQUISITES[$i]})" ]; then
	    echo " Error: ${PREREQUISITES[$i]} is not installed." >&2
	    echo " Please run 'sudo apt-get install ${PREREQUISITES[$i]}' first."
	    exit 1
	fi
    done

    if [ -x "$(command -v ${NRF_TOOL_NAME})" ]; then
	NRF_TOOL_INSTALLED=true
    fi   
    
    echo -e " Done. All prerequisites met (NRF toolchain" \
	 "installed:${NRF_TOOL_INSTALLED})."
}

show_welcome()
{
    echo -e "This script helps you to download the Nordic SDK you\n" \
    "need and update it with the BLEnd patch for continuous\n" \
    "neighbor discovery.\n"
}

version_select()
{
    echo -e "\n- Which SDK version do you need?"
    for (( i=0; i<${#SDK_ADDR[@]};i++ ));
    do
	echo "  [$i]:${SDK_NAMES[$i]}"
    done

    read -p " Enter the SDK# and press [ENTER]: " in_sdk_version

    if [ "$in_sdk_version" -ge 0 -a "$in_sdk_version" -le 2 ];
    then SDK_VERSION=$in_sdk_version;
    else echo "Invalid version selected."      
	 exit 1
    fi
}

download_sdk()
{
    URL="${SDK_ADDR[$SDK_VERSION]}"
    mkdir -p "${SDK_DIR}"
    ARCHIVE_NAME="${URL##*/}"
    echo "Downloading SDK [${SDK_NAMES[$SDK_VERSION]}]..."
    if [ -f "${SDK_DIR}/${ARCHIVE_NAME}" ]; then
    	echo " ${ARCHIVE_NAME} exists, skip downloading."
    else
    	echo " Downloading SDK (${URL}) to ${ROOT_DIR}/${SDK_DIR}"
    	wget --directory-prefix "${SDK_DIR}" --progress=bar "${URL}"
    fi

    dir_output_name="${ARCHIVE_NAME%.*}"
    if [ ${SDK_VERSION} -eq 2 ]; then
	dir_output_name=${THINGY_ARCHIVE_NAME}
    fi
    
    echo "Extracting SDK [${SDK_NAMES[$SDK_VERSION]}]"
    SDK_PATH="${SDK_DIR}/${dir_output_name}"
    if [ ! -d "${SDK_PATH}" ]; then
	unzip -o "${SDK_DIR}/$ARCHIVE_NAME" -d "${SDK_DIR}" | pv -l >/dev/null
	echo " Done."
    else
	echo " Directory exists, skip extracting."
    fi
}

add_blend_src()
{
    echo -e "\nAdding BLEnd protocol source to the SDK..."
    if [ ! -d "${BLEND_SRC_DIR}" ]; then
	echo " Error. Source directory doesn't exist."
	exit 1
    fi
    if [ ! -d "${SDK_PATH}" ]; then
	echo " Error. SDK directory doesn't exist."
	exit 1
    fi

    cp -r "${BLEND_SRC_DIR}" "${SDK_PATH}"

    if [ -d "${SDK_PATH}/blend" ]; then
	echo " Done."
    fi
}

add_template_project()
{
    echo -e "\nCreating template project..."
    if [ "$SDK_VERSION" -le 1 ];then
	proj_dir="${SDK_PATH}/examples/ble_central"
    else
	thingy_sdk_change
	proj_dir="${SDK_PATH}/project"
    fi

    if [ ! -d "$proj_dir" ]; then
	echo " Error. Cannot find SDK project directory(${proj_dir})"
	exit 1
    fi

    temp_dir="${BLEND_TEMP_ORI_LOCATION}/${SDK_NAMES[$SDK_VERSION]}/${BLEND_TEMP_NAME}"

    cp -r "${temp_dir}" "${proj_dir}"

    if [ -d "${proj_dir}/${BLEND_TEMP_NAME}" ]; then
	echo -e " Done. \n Template project location: \033[1m${proj_dir}/${BLEND_TEMP_NAME}\033[0m"
    fi
}

thingy_sdk_change()
{
    echo " Installing minor modifications in Thingy SDK..."
    for (( i=0; i<${#THINGY_SDK_CHANGES[@]};i++));
    do
	src_dir="${THINGY_SDK_MOD_DIR}/${THINGY_SDK_CHANGES[$i]}"
	cp -r "${src_dir}" "${SDK_PATH}"
    done
    echo "  Done."
}

finish()
{
    echo -e "\n \033[1mAll done. You're now ready to compile the SDK and begin your" \
	 "BLEnd development (with the template project).\033[0m"

    if [ ! "$NRF_TOOL_INSTALLED" == true ]; then
	echo -e "\n -[nRF5x Command Line Tools] is not installed. Follow ${TOOLCHAIN_URL}" \
	     " to install the nRF toolchain."
    fi

    if [ "${SDK_VERSION}" -eq 2 ]; then
	echo -e "\n (Guide for setting up Thingy SDK: ${COMPILE_THINGY_SDK_URL})"
	echo -e "\n (Guide for compiling Thingy firmware: ${COMPILE_THINGY_URL})"
    else
	echo -e "\n (Guide for compiling nRF5x SDK: ${COMPLIE_NRF52_URL})"
    fi

}

show_welcome

check_prerequisite

version_select

download_sdk

add_blend_src

add_template_project

finish
