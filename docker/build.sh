#!/bin/bash

ROS_DISTRO="humble"
BUILD_USER_ID=$(id -u)
BUILD_GROUP_ID=$(id -g)

path="."
image_name="oriens-image:latest"

if [ "${ROS_DISTRO}" = "humble" ]; then
	if ! docker build -t ${image_name} \
		--build-arg UID=${BUILD_USER_ID} \
		--build-arg GID=${BUILD_GROUP_ID} \
		${path} ; then
		exit 1
	fi
else
	echo "Unsupported ros distro"
	exit 1
fi

