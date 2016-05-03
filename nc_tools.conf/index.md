---
layout: page
title: nc_tools.conf - Common configuration file
permalink: /nc_tools.conf/
---

nc_tools.conf
=============

Project specific configuration shared by all nc_tools. Searched in current working directory and parent paths, allowing
the configuration to be project specific.

defaults
--------

```
default = {
    "machine" = "name",
    "units" = "metric"
}
```

machine
-------

```
machine = {}
machine.name = {
    units = "metric"
    type = "mill",
    spindle = {"100-1000", "2000-6000"}
}
```

machine.type

 - "mill" - Milling machine
 - "lathe" - Lathe

 machine.units

 - "metric" - Millimeter
 - "imperial" - Inch

machine.spindle

- array of rpm ranges
 - Can be specified as discreet steps, e.g. {"100", "200"}, as a range, e.g. {"100-200"}, or a combination, e.g. {"100",
   "200-300"}

tool_table
----------

```
tool_table = {
    [1] = {
        name = "1.5mm carbide end mill",
        length = 40,
        diameter = 1.5,
        flute_length = 10,
        shank_diameter = 3.175
    }
}
```
