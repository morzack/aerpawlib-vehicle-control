#!/usr/bin/bash

# assumes past installation using pip install -e .

rm -rf docs
pdoc --html aerpawlib
mv html/aerpawlib docs
rmdir html
