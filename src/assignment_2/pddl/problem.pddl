(define (problem explore_and_photograph)
  (:domain aruco_navigation)
  
  ;; World: simple_world.sdf from https://github.com/CarmineD8/exprob_assignment2
  ;; 
  ;; Environment: 20x20m area with walls at boundaries and inner obstacles
  ;; Inner walls at: (0,0), (3, 6.76), (-1.04, -8.58)
  ;;
  ;; ArUco marker positions:
  ;;   aruco_box:   (-8.13, 8.96)  -> NW corner
  ;;   aruco_box_1: (8.20, 9.01)   -> NE corner  
  ;;   aruco_box_2: (8.60, -8.90)  -> SE corner
  ;;   aruco_box_3: (-8.04, -8.85) -> SW corner
  ;;
  ;; Assignment waypoints (robot positions for marker detection):
  ;;   wp_nw: (-6.0, 6.0)  -> can see NW marker
  ;;   wp_ne: (6.0, 6.0)   -> can see NE marker
  ;;   wp_sw: (-6.0, -6.0) -> can see SW marker
  ;;   wp_se: (6.0, -6.0)  -> can see SE marker
  
  (:objects
    turtlebot - robot
    wp_center - waypoint
    wp_nw - waypoint  ;; Northwest (-6, 6) - detects marker at (-8.13, 8.96)
    wp_ne - waypoint  ;; Northeast (6, 6) - detects marker at (8.20, 9.01)
    wp_sw - waypoint  ;; Southwest (-6, -6) - detects marker at (-8.04, -8.85)
    wp_se - waypoint  ;; Southeast (6, -6) - detects marker at (8.60, -8.90)
    marker_0 - marker
    marker_1 - marker
    marker_2 - marker
    marker_3 - marker
  )
  
  (:init
    ;; Robot starts at center
    (robot_at turtlebot wp_center)
    (robot_available turtlebot)
    
    ;; Waypoint connectivity (fully connected for simplicity)
    (connected wp_center wp_nw)
    (connected wp_center wp_ne)
    (connected wp_center wp_sw)
    (connected wp_center wp_se)
    (connected wp_nw wp_center)
    (connected wp_ne wp_center)
    (connected wp_sw wp_center)
    (connected wp_se wp_center)
    (connected wp_nw wp_ne)
    (connected wp_ne wp_nw)
    (connected wp_nw wp_sw)
    (connected wp_sw wp_nw)
    (connected wp_ne wp_se)
    (connected wp_se wp_ne)
    (connected wp_sw wp_se)
    (connected wp_se wp_sw)
    
    ;; Marker locations (one marker per corner)
    ;; These will be discovered during exploration
    (marker_at marker_0 wp_nw)
    (marker_at marker_1 wp_ne)
    (marker_at marker_2 wp_sw)
    (marker_at marker_3 wp_se)
    
    ;; Marker ordering (will be set after detection based on actual IDs)
    ;; This is a template - the mission controller will update this
    (first_marker marker_0)
    (current_target marker_0)
    (next_marker marker_0 marker_1)
    (next_marker marker_1 marker_2)
    (next_marker marker_2 marker_3)
  )
  
  (:goal
    (and
      ;; All waypoints must be visited (for exploration)
      (waypoint_visited wp_nw)
      (waypoint_visited wp_ne)
      (waypoint_visited wp_sw)
      (waypoint_visited wp_se)
      
      ;; All markers must be detected and photographed
      (marker_detected marker_0)
      (marker_detected marker_1)
      (marker_detected marker_2)
      (marker_detected marker_3)
      (marker_photographed marker_0)
      (marker_photographed marker_1)
      (marker_photographed marker_2)
      (marker_photographed marker_3)
    )
  )
)
