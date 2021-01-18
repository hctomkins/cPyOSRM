# cPyOSRM

A simple (80 lines) C++ binding to use the libosrm API directly in python. Can handle significantly larger requests than the HTTP OSRM API. Zero guarantees made about memory safety, or good C++.

I've been using this personally, but I thought I'd share the pattern for anyone else interested! Currently only implements `table`, 
but other functions can be implemented easily following the same pattern.

#### Usage:
```$xslt
from PyOSRM import PyOSRM
import numpy as np

osrm = PyOSRM()

lons = [...] # list of float lons
lats = [...] # list of float lats
sources = [...] # list of integer indices into lons/lats to use as sources
dests = [...] # list of integer indices into lons/lats to use as destinations
output = np.array((len(sources),len(dests)),dtype=np.float32)

osrm.table(lons, lats, sources, dests, output)

print(output)

```
#### Installation

- Build+Install `libosrm`, place `eigen3` in include path
- Clone `pybind11` into directory
- Adjust hardcoded osrm routing path in `main.cpp`
- Adjust python installation path in `cmakelists.txt`
- Configure + Build + Install with cmake.