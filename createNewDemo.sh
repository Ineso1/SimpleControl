#!/bin/bash

# Ensure a project name is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <NewProjectName>"
    exit 1
fi

NEW_PROJECT_NAME=$1

# Ensure FLAIR_ROOT is set
if [ -z "$FLAIR_ROOT" ]; then
    echo "Environment variable FLAIR_ROOT is not set. Please set it and try again."
    exit 1
fi

# Create myDemos directory if it doesn't exist
MY_DEMOS_DIR="$FLAIR_ROOT/flair-src/myDemos"
if [ ! -d "$MY_DEMOS_DIR" ]; then
    echo "Creating directory $MY_DEMOS_DIR..."
    mkdir -p "$MY_DEMOS_DIR"
fi

# Navigate to myDemos directory
cd "$MY_DEMOS_DIR" || { echo "Failed to navigate to $MY_DEMOS_DIR"; exit 1; }

# Clone the new project
echo "Cloning the CircleFollower demo into $NEW_PROJECT_NAME..."
"$FLAIR_ROOT/flair-src/scripts/clone_demo.sh" "$FLAIR_ROOT/flair-src/demos/CircleFollower/" "$NEW_PROJECT_NAME"
if [ $? -ne 0 ]; then
    echo "Cloning the demo failed. Exiting..."
    exit 1
fi

# Update or create CMakeLists.txt
CMAKE_FILE="$MY_DEMOS_DIR/CMakeLists.txt"
if [ ! -f "$CMAKE_FILE" ]; then
    echo "Creating new CMakeLists.txt..."
    cat <<EOF > "$CMAKE_FILE"
PROJECT(FlairDemos)
cmake_minimum_required(VERSION 2.8)
include(\$ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
EOF
fi

# Add the new project to CMakeLists.txt if not already present
if ! grep -q "add_subdirectory(\${CMAKE_CURRENT_SOURCE_DIR}/$NEW_PROJECT_NAME/)" "$CMAKE_FILE"; then
    echo "Adding $NEW_PROJECT_NAME to CMakeLists.txt..."
    echo "add_subdirectory(\${CMAKE_CURRENT_SOURCE_DIR}/$NEW_PROJECT_NAME/)" >> "$CMAKE_FILE"
else
    echo "$NEW_PROJECT_NAME already exists in CMakeLists.txt."
fi

# Navigate to the build directory
echo "Navigating to the build directory..."
cd "$FLAIR_ROOT/flair-build/build" || { echo "Failed to navigate to build directory"; exit 1; }

# Build the project
echo "Running 'make install -j12'..."
make install -j12
if [ $? -ne 0 ]; then
    echo "'make install' failed. Exiting..."
    exit 1
fi

# Run the new project
echo "Running 'flairrun $NEW_PROJECT_NAME'..."
flairrun "$NEW_PROJECT_NAME"
if [ $? -ne 0 ]; then
    echo "'flairrun $NEW_PROJECT_NAME' failed."
    exit 1
fi

echo "Script completed successfully."
