#!/bin/bash

for f in $( find *.py -prune ); do
  echo "====================="
  echo $f
  python $f
done

exit 0
