# Vendored frontier explorer

`explore` (package `explore_lite`) and `explore_lite_msgs` are from
https://github.com/robo-friends/m-explore-ros2 at
326cf8a0b487c34246bb8f3326afbcd69576dc60 (BSD; see LICENSE.m-explore-ros2).
Only the explorer packages are included, not map_merge or another Nav2 stack.

Local integration changes: optional collision-checked initial Nav2 scan rotation;
transient-local costmap subscription; shutdown-aware startup; explicit action
dependencies; serialized navigation with configurable displacement/distance
progress monitoring; optional total navigation timeout (disabled by default).
Known-free frontier endpoints have minimum travel distance, obstacle clearance,
and boundary setback, then Nav2 ComputePathToPose validates reachability.
Spatial endpoint blacklisting handles rejection, failure, and stalls; successful
endpoints are remembered separately. Five exhausted searches precede completion.
Upstream frontier search and scoring are retained. Concise progress/status logs
and regression tests cover long progressing goals, oscillation, stalls,
cancellation, and continuation to another goal.
