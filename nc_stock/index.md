---
layout: page
title: nc_stock - Generate stock model
permalink: /nc_stock/
---

Generate a stock model from given shape and dimensions.

```
$ nc_stock --help
nc_stock:
  -h [ --help ]         display this help and exit
  --box                 Box stock shape
  -X [ --x0 ] arg (=0)  X0 dimension
  -Y [ --y0 ] arg (=0)  Y0 dimension
  -Z [ --z0 ] arg (=0)  Z0 dimension
  -x [ --x1 ] arg       X1 dimension
  -y [ --y1 ] arg       Y1 dimension
  -z [ --z1 ] arg       Z1 dimension
```

```
$ nc_stock --box -x50 -y50 -z-10 > model.off
$ nc_backplot --model model.off
```

![model image]({{ site.baseurl }}/assets/nc_stock.png)
