# Path where the slider mesh is located
root_dir="/home/rise/catkin_ws/src/stable-pushnet-datagen/scripts/assets/dish_mesh"

# Absolute path to this script
SCRIPT_PATH=$(readlink -f "$0")
# Absolute path to the directory where this script is located
SCRIPT_DIR=$(dirname "$SCRIPT_PATH")
# Path where the asset (slider urdf) will be located
asset_dir="$SCRIPT_DIR/assets/dish_urdf"

mesh_exts=($(ls "$SCRIPT_DIR/assets/dish_mesh"))

# Iterate over the array and run the Python script with different arguments
for mesh_ext in "${mesh_exts[@]}"
do
    python3 mesh_to_urdf_test.py --root $root_dir --target $asset_dir --mesh_name $mesh_ext
done