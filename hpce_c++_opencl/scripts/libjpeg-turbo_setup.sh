#!/bin/bash
MAKE_DIR=$(/bin/pwd)
echo "Installing in $MAKE_DIR"
SRC_URL="http://www.doc.ic.ac.uk/~cyl312/HPCE/files"
mkdir -p $MAKE_DIR/packages
curl $SRC_URL/libjpeg-turbo-1.4.2.tar.gz -o $MAKE_DIR/packages/libjpeg-turbo-1.4.2.tar.gz
mkdir -p $MAKE_DIR/source
cd $MAKE_DIR/source
tar -xvf $MAKE_DIR/packages/libjpeg-turbo-1.4.2.tar.gz
cd $MAKE_DIR/source/libjpeg-turbo-1.4.2
mkdir -p $MAKE_DIR/build
cd $MAKE_DIR/build
sh $MAKE_DIR/source/libjpeg-turbo-1.4.2/configure --prefix=$MAKE_DIR/bin/lib/libjpeg-turbo
make
mkdir -p $MAKE_DIR/bin/lib/libjpeg-turbo
make install prefix=$MAKE_DIR/bin/lib/libjpeg-turbo libdir=$MAKE_DIR/bin/lib/libjpeg-turbo/lib
echo "Installed libjpeg-turbo locally in $MAKE_DIR"



