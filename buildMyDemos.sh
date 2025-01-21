#!/bin/bash

# Navigate to the build directory
cd $FLAIR_ROOT/flair-build/build || { echo "Failed to navigate to build directory"; exit 1; }

# Run 'make install' with 12 parallel jobs
echo "Running 'make install -j12'..."
make install -j12
if [ $? -ne 0 ]; then
    echo "'make install' failed. Exiting..."
    exit 1
fi

# If everything is fine, run 'flairrun SimpleControl'
echo "Running 'flairrun SimpleControl'..."
flairrun SimpleControl
if [ $? -ne 0 ]; then
    echo "'flairrun SimpleControl' failed."
    exit 1
fi

echo "Script completed successfully."
