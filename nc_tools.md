 * nc_backplot
    * display a backplot of the incoming gcode - no output
    * backplot renamed to nc_backplot
 * nc_shortlines
    * split incoming gcode into short line segments
    * cli option --arc-only
 * nc_delay
    * delay the output of the gcode based on the machining time option 1x 2x, etc
 * nc_model
    * generate a 3d model of the incoming gcode, tools and stock
 * nc_rename_axis
    * rename an axis, a=> is delete, a=>b is rename, a<=>b is swap
 * nc_fold_moves
    * fold adajacent G00 moves
 * nc_pick
    * nc_stop --at-comment blah --at-line 205 --at-tool 3
    * selectively pick gcode from file
    * e.g. within bounding box
 * nc_sim
    * simulated model on display
 * nc_bounds
    * bounding box
 * nc_scale
    * scale gcode axes up or down
 * nc_transform
    * transform / rotate stock for next operation

Generators
 * lathe_path
    * generate lathe operations for svg path
    * TODO determine machining strategy for lathe operations based on different tools and back faces
 * mill_path
    * generate profile operation for svg path
    * pick from SVG file with xpath pattern
    * opt. scale / transform
    * profile / pocket
