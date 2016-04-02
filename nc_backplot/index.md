---
layout: page
title: nc_backplot - GCode backplotter
permalink: /nc_backplot/
---

Provide a visual representation of the GCode provided on standard input.

Optionally allows an .off format 3d model to be overlaid in the view window

```
$ nc_backplot --help
nc_backplot:
  -h [ --help ]         display this help and exit
  --model arg           Model file
```

`$ nc_backplot --model motor_mount.op0.off < motor_mount.op0.ngc`

![backplot image]({{ site.baseurl }}/assets/nc_backplot.png)
