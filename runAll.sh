resize -s 40 172

if [ -d "build" ]; then
    cd build
fi

allExecsCount=$(find . -executable -type f ! -iname "*.so" | wc -l)

let count=0
directoriesList=($(ls -d */))

for dir in ${directoriesList[@]}; do
    if [ -d $dir ]; then

        cd $dir
        currentDirExecs=$(find . -executable -type f ! -iname "*.so")

        for exe in ${currentDirExecs[@]}; do
            count=$((count + 1))
            echo $exe

            if [ $count -lt $allExecsCount ]; then
                $exe &
            else
                $exe
            fi
        done
        cd ..
    fi
done

