cd build

allExecs=$(find . -executable -type f ! -iname "*.so" -printf "%f\n")
for exe in ${allExecs[@]}; do
    if [ ${#exe} -gt 15 ]; then
        exe="${exe:0:15}"
    fi
    echo $exe
    pkill $exe
done

cd ..
