# 编译nabo_core
cd nabo_core
if [ -d "build" ]; then
    cd build 
else
    mkdir build && cd build
fi
cmake .. && make
ls
cd ../..
ls
# 编译nabo_mujoco
cd nabo_mujoco
ls
if [ -d "build" ]; then
    cd build 
else
    mkdir build && cd build
fi
cmake .. && make
#拷贝并运行main
cp ../../nabo_core/build/libnabo.so ../../nabo_mujoco/nabo
cp ../../nabo_core/build/libnabo.so ../../nabo_mujoco/build
./main
