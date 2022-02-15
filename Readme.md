# <img src="docs/images/overworld.png" width="150">

## Install bullet

```
cmake .. -DCMAKE_INSTALL_PREFIX=your/install/path -DBUILD_SHARED_LIBS=ON
make install
```

then

```
export BULLET_INSTALL_PATH=your/install/path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BULLET_INSTALL_PATH/lib
```