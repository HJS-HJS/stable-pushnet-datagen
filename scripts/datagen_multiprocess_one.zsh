#!/bin/zsh

# Absolute path to this script
SCRIPT_PATH=$(readlink -f "$0")

# Absolute path to the directory where this script is located
SCRIPT_DIR=$(dirname "$SCRIPT_PATH")

asset_dir="$SCRIPT_DIR/assets/dish_urdf"
config_dir="$SCRIPT_DIR/../config/config_pushsim.yaml"

# Get the list of file names in the directory
option2_args=($(ls $asset_dir))

# Iterate over the array and run the Python script with different arguments
for arg in "${option2_args[@]}"
do
    if [ $arg = '64ccb121147c56af48ce81ccd3000f65' ] ; then
        sudo pkill python3
        python3 generate_train_data.py --config $config_dir --save_results --slider_name $arg
    fi
done