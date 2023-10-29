CURRENT_FOLDER=$PWD

cd /ros_ws
colcon build --symlink-install "$@" || true
if [ -f "install/setup.bash" ]
then
    source install/setup.bash
fi
cd $CURRENT_FOLDER