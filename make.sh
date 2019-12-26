mkdir out
make -j4
make modules -j4
make install INSTALL_PATH=./out
make modules_install INSTALL_MOD_PATH=./out

