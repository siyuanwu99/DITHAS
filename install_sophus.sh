#!/bin/bash


INSTALL_DIR=$HOME/Softwares/
TMP_DIR=$HOME/Softwares/tmp


# Function to install LKH
install_lkh() {
	echo "[STAR Lab] Installing LKH to $TMP_DIR..."

	# Create a temporary directory and navigate to it
	mkdir -p "$TMP_DIR" && cd "$TMP_DIR"

	# Download and extract the package
	wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz
	tar xvfz LKH-3.0.6.tgz

	# Navigate to the extracted directory
	cd LKH-3.0.6

	# Compile the software
	make

	echo "$PASSWORD" | sudo -S cp LKH /usr/local/bin

	# Clean up temporary directory
	rm -rf "$TMP_DIR"

	echo "[STAR Lab] LKH has been successfully installed!"
}


install_sophus () {
  echo "[Script] Installing Sophus to $TMP_DIR..."
  sudo apt-get install libeigen3-dev ros-noetic-sophus -y
}


