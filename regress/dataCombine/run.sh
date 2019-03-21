if [ -e build ]
then
  rm -r build
fi

mkdir build
cd build

cmake ..
make

cd ..
cp *.txt build/bin
cd build

cd bin
./main
