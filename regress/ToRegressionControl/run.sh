if [ -e build ]
then
  rm -r build
fi

mkdir build
cp -r originalData build
#cp data.txt build
#cp cluster.txt build
cd build

cmake ..
make

./main
