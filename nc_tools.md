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
 * lathe_path
    * generate lathe operations for svg path
    * TODO determine machining strategy for lathe operations based on different tools and back faces
 * nc_fold_moves
    * fold adajacent G00 moves
 * nc_stop
    * nc_stop --at-comment blah --at-line 205 --at-tool 3
    * stop the output after some condition
 * nc_sim
    * simulated model on display
