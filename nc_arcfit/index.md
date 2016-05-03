---
layout: page
title: nc_arcfit - GCode arc fitting
permalink: /nc_arcfit/
---

Recover G2/G3 arcs from line segments with tolerance.

Note: This tool will behave eratically if the input gcode uses cutter compensation. The re-written G2/G3 arcs will be output with cutter comp applied, while the remaining gcode is unmodified.


```
$ nc_arcfit --help
nc_arcfit:
  -h [ --help ]                         display this help and exit
  -c [ --chord_height ] arg (=0.10000000000000001)
                                        Chord height tolerance
  -r [ --radius_dev ] arg (=0.10000000000000001)
                                        Radius deviation tolerance
  -p [ --planar_dev ] arg (=9.9999999999999995e-07)
                                        Planar deviation tolerance
  -t [ --theta_min ] arg (=0.19625000000000001)
                                        Minimum arc theta

base options:
  --config arg                          Configuration file
  --machine arg                         Machine configuration

```

```
$ nc_backplot < circle.ngc 
G00 X5
G01 X4.842916 Y1.243449 F50
    X4.381533 Y2.408768
    X3.644843 Y3.422736
    X2.679134 Y4.22164
    X1.545085 Y4.755283
    X0.313953 Y4.990134
    X-0.936907 Y4.911436
    X-2.128896 Y4.524135
    X-3.18712 Y3.852566
    X-4.045085 Y2.938926
    X-4.648882 Y1.840623
    X-4.960574 Y0.626666
    Y-0.626666
    X-4.648882 Y-1.840623
    X-4.045085 Y-2.938926
    X-3.18712 Y-3.852566
    X-2.128896 Y-4.524135
    X-0.936907 Y-4.911436
    X0.313953 Y-4.990134
    X1.545085 Y-4.755283
    X2.679134 Y-4.22164
    X3.644843 Y-3.422736
    X4.381533 Y-2.408768
    X4.842916 Y-1.243449
    X5 Y0
G00 X0
```

![backplot image]({{ site.baseurl }}/assets/nc_arcfit_src.png)

```
$ nc_arcfit < circle.ngc 
G0 X5 
G3 X5.000001 Y0 I-5 J0 F50 
G0 X0 
```

