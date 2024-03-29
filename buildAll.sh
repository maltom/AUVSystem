modulesList=($(ls -d */))

if [ ! -d "build" ]; then
    mkdir build
fi
cd build

cmake -DCMAKE_BUILD_TYPE=Debug ..
# cmake -DCMAKE_BUILD_TYPE=Release ..
make -j12
unset GLOBIGNORE

for index in ${modulesList[@]}; do
    ignored=${index%/*}
    GLOBIGNORE=$GLOBIGNORE:$ignored
done

additional+="Commons"

GLOBIGNORE=$GLOBIGNORE:$additional

rm -r *

GLOBIGNORE=$GLOBIGNORE:*.so

for index in ${modulesList[@]}; do
    if [ -d $index ]; then

        cd $index
        rm -r *
        cd ..
    fi
done

unset GLOBIGNORE
