---
layout: page
title: nc_contour_pocket - Contour offset pocket
permalink: /nc_contour_pocket/
---

Generate parallel contour offset toolpaths from an input (closed) path.

```
$ nc_contour_pocket --help
nc_contour_pocket:
  -h [ --help ]         display this help and exit
  -r [ --tool_r ] arg   Tool radius

```

```
$ nc_backplot < profile.ngc
G02 X-8 Y8 J8 F500
G01 X-8 Y92
G02 X0 Y100 I8
G01 X184 Y100 F500
G02 X192 Y92 J-8
G01 X192 Y8
G02 X184 Y0 I-8
G01 X0 Y0
```

![backplot image]({{ site.baseurl }}/assets/nc_contour_pocket_profile.png)

```
$ nc_contour_pocket -r 4 < profile.ngc | nc_backplot
...
```
![backplot image]({{ site.baseurl }}/assets/nc_contour_pocket.png)
