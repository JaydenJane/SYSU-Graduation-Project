if [ -e build ]
then
    cd build
    ./main
else
    echo "The project shoulb be built!"
    echo "Please run the script -run.sh-!"
fi

