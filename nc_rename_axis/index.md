---
layout: page
title: nc_rename_axis - delete/rename axes
permalink: /nc_rename_axis/
---

Swap, rename, and delete individual axis words from GCode motion commands.


```
$ nc_rename_axis --help
nc_rename_axis:
  -h [ --help ]         display this help and exit
  -d [ --delete ] arg   delete axis [XYZABC]
  -s [ --swap ] arg     swap axes [XYZABC][XYZABC]
```

```
$ cat profile.ngc
G1 F50 X0Z0
G0 X4 Z1.0
G0 X1.6
G1 Z-4.0 
X2.5 Z-5.0
G1 X4

$ nc_rename_axis --swap ZY < profile.ngc
G1 X0 Y0 F50 
G0 X4 Y1 
G0 X1.6 
G1 Y-4 
X2.5 Y-5 
G1 X4 
```

