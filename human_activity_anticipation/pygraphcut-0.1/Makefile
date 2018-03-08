# Copyright 2007 Thomas Finley, tfinley@gmail.com

PYTHON := python
LIB_TARGET := graphcut.so
SYNCER := rsync -rvtu --exclude 'build' --exclude '*~' --exclude $(LIB_TARGET)
FULLNAME := py$(shell $(PYTHON) setup.py --fullname)

.PHONY : all dist install test clean cleaner

all: $(LIB_TARGET)

graphcut.so:
	$(PYTHON) setup.py build
	rm -rf $(LIB_TARGET)
	ln -s build/lib.*/$(LIB_TARGET)

html/graphcut_code.html: $(LIB_TARGET)
	pydoc -w graphcut
	mv graphcut.html $@

README.txt: html/readme.html
	links -dump $< > $@

install: all
	$(PYTHON) setup.py install

test: all
	$(PYTHON) test.py

clean:
	rm -rf build
	rm -rf $(LIB_TARGET)

cleaner: clean
	rm -rf dist
	find . -name "*~" -exec rm {} \;

$(FULLNAME).tar.bz2: README.txt cleaner
	mkdir -p dist/$(FULLNAME)
	cp -rp html src Makefile *.{py,supp,txt} dist/$(FULLNAME)
	cd dist; tar -jcf $@ $(FULLNAME); mv $@ ..
	rm -rf dist

dist: $(FULLNAME).tar.bz2
