nc_tools
========

A collection of composable CNC/CAM GCode tools.

```
$ cat robot2.ngc | nc_backplot
```

![robot backplot](https://raw.github.com/mythagel/backplot/master/robot_backplot.png)

Implemented utilities
=====================

 * [nc_backplot](http://mythagel.github.io/nc_tools/nc_backplot/)
    * display a backplot of the incoming gcode
 * [nc_model](http://mythagel.github.io/nc_tools/nc_model/)
    * generate a 3d model of the incoming gcode, given tools and stock
 * [nc_rename_axis](http://mythagel.github.io/nc_tools/nc_rename_axis/)
    * rename an axis, a=> is delete, a=>b is rename, a<=>b is swap
 * nc_bounds
    * bounding box of gcode or .off model
 * nc_transform
    * transform / rotate stock for next operation
    * nc_transform -a 45 -b 45 -x -30 # rotate 45 deg around x, 45 around y, translate -30 on x
 * nc_tooltable
    * generate linuxcnc tool table from nc_tools.conf
 * [nc_contour_pocket](http://mythagel.github.io/nc_tools/nc_contour_pocket/)
    * Generate contour parallel toolpath from gcode path
 * [nc_stock](http://mythagel.github.io/nc_tools/nc_stock/)
    * Generate .off stock models
 * [nc_svgpath](http://mythagel.github.io/nc_tools/nc_svgpath/)
    * SVG path data -> gcode converter
 * nc_identity
    * gcode identity transformation - base for new tools
 * [nc_arcfit](http://mythagel.github.io/nc_tools/nc_arcfit/)
    * recover G2/G3 arcs from linear paths
 * nc_delay
    * delay the output of the gcode based on the machining time option 1x 2x, etc
 * nc_shortlines
    * split incoming gcode into short line segments
    * ~~cli option --arc-only~~


~~not implemented / not complete~~

 * ~~nc_fold_moves~~
    * fold adajacent G00 moves
 * ~~nc_pick~~
    * nc_stop --at-comment blah --at-line 205 --at-tool 3
    * selectively pick gcode from file
    * e.g. within bounding box
 * ~~nc_inspect~~
    * allow playback of the gcode
    * feed rate override + pause
    * seek forward and backward
    * zoom to section of timeline
 * ~~nc_scale~~
    * scale gcode axes up or down
 * ~~nc_offset_rotational_origin~~
    * translate to/from tool local (canonical) rotations to world rotation (e.g. rotary tables)
 * ~~nc_feedrate~~
    * analyse / optimise feedrate of provided gcode
 * ~~nc_lathe_roughing~~
    * G70/G71 canned cycle style lathe roughing generator 
 * ~~nc_blueprint~~
    * Generate SVG blueprint with 3 ortho + perspective view of gcode / model.

Generators
==========

 * ~~lathe_path~~
    * generate lathe operations for svg path
    * TODO determine machining strategy for lathe operations based on different tools and back faces
 * ~~mill_path~~
    * generate profile operation for svg path
    * pick from SVG file with xpath pattern
    * opt. scale / transform
    * profile / pocket
