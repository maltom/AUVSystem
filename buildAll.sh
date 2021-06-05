modulesList=($(ls -d */))

if [ ! -d "build" ]; then
    mkdir build
fi
cd build

cmake ../.
make -j12
unset GLOBIGNORE

for index in ${modulesList[@]}; do
    ignored=${index%/*}
    GLOBIGNORE=$GLOBIGNORE:$ignored
done

rm -r *

GLOBIGNORE=build

for index in ${modulesList[@]}; do
    if [ -d $index ]; then
        cd $index
        rm -r *
        cd ..
    fi
done
