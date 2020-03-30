BASE_DIR="BARC_research"

if [ ! -d ~/$BASE_DIR/arduino/.arduino_nano328_node ]; then
    mkdir -p ~/$BASE_DIR/arduino/.arduino_nano328_node/src
fi

if [ ! -L ~/$BASE_DIR/arduino/.arduino_nano328_node/lib ]; then
    ln -s ~/$BASE_DIR/arduino/sketchbook/libraries ~/$BASE_DIR/arduino/.arduino_nano328_node/lib
fi

cd ~/$BASE_DIR/arduino/.arduino_nano328_node
cp ../arduino_nano328_node/arduino_nano328_node.ino src/;
#ano clean; ano build -m nano328;
#ano upload -m nano328 -p /dev/ttyUSB0;
ano clean; ano build -m mega2560;
ano upload -m mega2560 -p /dev/ttyACM0; 
cd -