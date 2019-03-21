if [ -e build ]
then
  rm -r build
fi

mkdir build
cd build

cmake ..
make

./main
