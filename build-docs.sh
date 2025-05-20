#!/usr/bin/bash

# assumes past installation using pip install -e .

rm -rf docs
pdoc -o ./html aerpawlib
mkdir -p docs
mv html/* docs
rmdir html
