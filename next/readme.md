For non qt users, open the file ./install.sh and edit your linuxcnc path.
Then run the file to create the libnext.so shared library.

$ ./install.sh

The libnext.so library will be copied to your linuxcnc/rtlib.
Then the script tell's linux where to find the library with the ldconfig command.
