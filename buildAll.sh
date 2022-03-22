params="${@}"

buildParams=""
buildTypeIsSet=false

for index in ${params[@]}; do
case $index in

  "NOLQR")
    buildParams=$buildParams"-DNOLQR=ON "
    ;;

  "SIMULATION")
    buildParams=$buildParams"-DSIMULATION=ON "
    ;;

  "NOSTM")
    buildParams=$buildParams"-DSIMULATION=ON "
    ;;

  "SIMULATION")
    buildParams=$buildParams"-DSIMULATION=ON "
    ;;

  "RELEASE")
  if [ $buildTypeIsSet = false ]; then
      buildParams=$buildParams"-DCMAKE_BUILD_TYPE=Release "
      buildTypeIsSet=true
  else
  echo "Set proper build type (RELEASE or DEBUG, you have to use one and only one of them)"
  exit 3
  fi
    ;;

  "DEBUG")
  if [ $buildTypeIsSet = false ]; then
    buildParams=$buildParams"-DCMAKE_BUILD_TYPE=Debug "
    buildTypeIsSet=true
  else
  echo "Set proper build type (RELEASE or DEBUG, you have to use one and only one of them)"
  exit 3
  fi
    ;;

  *)
    echo "Unknow parameter: $index"
    exit 2
    ;;
esac
done

if [ $buildTypeIsSet = false ]; then
  buildParams=$buildParams"-DCMAKE_BUILD_TYPE=Release "
fi

echo "Building with parameters" $buildParams

modulesList=($(ls -d */))

if [ ! -d "build" ]; then
    mkdir build
fi
cd build

cmake $buildParams ..

jobs=$(nproc --all)

make -j$jobs
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
