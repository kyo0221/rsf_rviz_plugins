# rsf_rviz_plugins

RViz plugins for the RSF robot. The waypoint panel sets the `start_index`
parameter of `/waypoint_navigator` from its spin box and calls
`/waypoint_navigator/start`, so a route can be started or resumed from any waypoint.
The spin box follows the navigator's `~/status` (current waypoint and total) unless
it is being edited. The RsfStatus panel shows the device state, GNSS quality and the
localization filter state (`/rsf_filter/state`: init / fix / dr / mcl).

Build this package in a ROS 2 Jazzy workspace together with
`rsf_navigation_executor`.

