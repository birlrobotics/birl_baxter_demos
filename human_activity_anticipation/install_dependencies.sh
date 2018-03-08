set -e
wget http://ftp.gnu.org/gnu/glpk/glpk-4.45.tar.gz
tar -xvzf glpk-4.45.tar.gz
cd glpk-4.45/
./configure
make
sudo make install
cd ..

wget ftp://ftp.gmplib.org/pub/gmp-5.0.2/gmp-5.0.2.tar.bz2
tar -xjvf gmp-5.0.2.tar.bz2
cd gmp-5.0.2
./configure
make
sudo make install
cd ..

wget http://tfinley.net/software/pyglpk/pyglpk-0.3.tar.bz2
tar -xjvf pyglpk-0.3.tar.bz2
# change -m32 to -m64 in setup.py
# comment the following line if running on a 32 bit machine
cp pyglpk_setup.py pyglpk-0.3/setup.py
cd pyglpk-0.3
make
sudo make install 
cd ..

wget http://pypi.python.org/packages/source/b/bitarray/bitarray-0.3.5.tar.gz#md5=322a25e04e7aece5e028e5b068329291
tar -xzvf bitarray-0.3.5.tar.gz
cd bitarray-0.3.5
sudo python setup.py install
cd ..

cd pygraphcut-0.1/
make
sudo make install
cd ..

sudo apt-get install python-numpy python-scipy


export PYTHONPATH=$PYTHONPATH:/home/nav/pkg/scene_labelling/svm-python-v204/pyobjs/build/lib.linux-x86_64-2.6/
