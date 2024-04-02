command=$1

if [ "$command" == "clean" ]; then
    rm -r nabo_core/build/ nabo_mujoco/build/ nabo_core/nabo_output/
    echo "# clean: finished successfully"
    exit 0
fi

# 编译nabo_core
cd nabo_core
if [ -d "build" ]; then
    cd build 
else
    mkdir build && cd build
fi
cmake .. && make -j8
if [ $? != 0 ]; then
    echo "# ERROR: build failed"
    exit 1
fi

cd ../..
# 编译nabo_mujoco
cd nabo_mujoco
if [ -d "build" ]; then
    cd build 
else
    mkdir build && cd build
fi
cmake .. && make -j8
if [ $? != 0 ]; then
    echo "# ERROR: build failed"
    exit 1
fi

#拷贝并运行main
cp ../../nabo_core/build/libnabo.so ../../nabo_mujoco/nabo
cp ../../nabo_core/build/libnabo.so ../../nabo_mujoco/mujoco
cp ../../nabo_core/build/libnabo.so ../../nabo_mujoco/build
./main
