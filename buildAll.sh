modulesList=($(ls -d */))

if [ ! -d "build" ]; then
    mkdir build
fi
cd build

cmake ../.
make
unset GLOBIGNORE

for index in ${modulesList[@]}; do
    ignored=${index%/*}
    GLOBIGNORE=$GLOBIGNORE:$ignored
done

rm -rv *

GLOBIGNORE=build

for index in ${modulesList[@]}; do
    if [ -d $index ]; then
        cd $index
        rm -rv *
        cd ..
    fi
done
