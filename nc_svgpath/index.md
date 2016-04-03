---
layout: page
title: nc_svgpath - SVG path to GCode
permalink: /nc_svgpath/
---

Converts SVG Tiny path data to gcode.

SVG path data support (elliptical arc data) is planned but incomplete.


```
$ nc_svgpath --help
nc_svg_path:
  -h [ --help ]         display this help and exit
  -f [ --feedrate ] arg cutting feed rate
```

```
$ cat triangle.path 
M 100 100 L 300 100 L 200 300 z
```

```
$ nc_svgpath -f50 < triangle.path | nc_backplot 
G00 X100 Y100
G01 X300 Y100 F50
G01 X200 Y300 F50
G01 X100 Y100 F50
```

![backplot image]({{ site.baseurl }}/assets/nc_svgpath.png)
