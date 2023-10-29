echo "alias pkg-build-up-to='bash $(pwd)/.devcontainer/scripts/pkg_build.sh --packages-up-to'" >> ~/.bashrc 
echo "alias pkg-build-select='bash $(pwd)/.devcontainer/scripts/pkg_build.sh --packages-select'" >> ~/.bashrc 
sudo chmod 777 $(pwd)/..