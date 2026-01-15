(define (domain aruco_navigation)
  (:requirements :strips :typing :durative-actions)
  
  (:types
    robot
    waypoint
    marker
  )
  
  (:predicates
    ;; Robot location
    (robot_at ?r - robot ?wp - waypoint)
    
    ;; Marker location (discovered during exploration)
    (marker_at ?m - marker ?wp - waypoint)
    
    ;; Waypoint connectivity for navigation
    (connected ?wp1 - waypoint ?wp2 - waypoint)
    
    ;; Waypoint visited for scanning
    (waypoint_visited ?wp - waypoint)
    
    ;; Marker states
    (marker_detected ?m - marker)
    (marker_photographed ?m - marker)
    
    ;; Robot can move (not busy)
    (robot_available ?r - robot)
    
    ;; Marker ordering (for visiting in order)
    (next_marker ?m1 - marker ?m2 - marker)
    (first_marker ?m - marker)
    (current_target ?m - marker)
    (all_markers_found)
  )
  
  ;; Move robot from one waypoint to another
  (:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration (= ?duration 10)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (robot_available ?r))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at start (not (robot_available ?r)))
      (at end (robot_at ?r ?to))
      (at end (robot_available ?r))
    )
  )
  
  ;; Scan for markers at current waypoint
  (:durative-action scan_waypoint
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (robot_available ?r))
      (at start (not (waypoint_visited ?wp)))
    )
    :effect (and
      (at start (not (robot_available ?r)))
      (at end (robot_available ?r))
      (at end (waypoint_visited ?wp))
    )
  )
  
  ;; Detect a marker at a waypoint (marker becomes known)
  (:durative-action detect_marker
    :parameters (?r - robot ?m - marker ?wp - waypoint)
    :duration (= ?duration 3)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (robot_available ?r))
      (at start (marker_at ?m ?wp))
      (at start (not (marker_detected ?m)))
    )
    :effect (and
      (at start (not (robot_available ?r)))
      (at end (robot_available ?r))
      (at end (marker_detected ?m))
    )
  )
  
  ;; Photograph a marker (approach and take picture)
  (:durative-action photograph_marker
    :parameters (?r - robot ?m - marker ?wp - waypoint)
    :duration (= ?duration 8)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (robot_available ?r))
      (at start (marker_at ?m ?wp))
      (at start (marker_detected ?m))
      (at start (current_target ?m))
      (at start (not (marker_photographed ?m)))
    )
    :effect (and
      (at start (not (robot_available ?r)))
      (at end (robot_available ?r))
      (at end (marker_photographed ?m))
      (at end (not (current_target ?m)))
    )
  )
  
  ;; Set next target marker after photographing current one
  (:durative-action set_next_target
    :parameters (?m1 - marker ?m2 - marker)
    :duration (= ?duration 1)
    :condition (and
      (at start (marker_photographed ?m1))
      (at start (next_marker ?m1 ?m2))
      (at start (marker_detected ?m2))
      (at start (not (current_target ?m2)))
    )
    :effect (and
      (at end (current_target ?m2))
    )
  )
)
