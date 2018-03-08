PYTHON := python

MODNAME := svmapi

all:
	$(PYTHON) setup.py build
	rm -f $(MODNAME).so
	ln -s build/lib.*/$(MODNAME).so

install:
	$(PYTHON) setup.py install

clean:
	rm -rf build
	rm -f svmapi.so
