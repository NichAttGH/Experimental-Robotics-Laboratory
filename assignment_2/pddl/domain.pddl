(define (domain motion)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents)


(:types 	robot waypoint marker)


(:predicates
		(robot_at ?r - robot ?wp - waypoint)
		(found_marker ?m - marker)
		(at_marker ?m - marker ?wp - waypoint)
		(back_home ?wp - waypoint)
	      
)

(:functions      
		(marker-counter)
)

(:durative-action go-home
		:parameters (?r - robot ?from - waypoint ?to - waypoint)
		:duration (= ?duration 30)
		:condition (and (at start (robot_at ?r ?from))
				(at start (= (marker-counter) 4)))
	        :effect (and 
				(at start (not (robot_at ?r ?from))) 
				(at end (robot_at ?r ?to))
				(at end (back_home ?to)))
)

(:durative-action goto
		:parameters (?r - robot ?from ?to - waypoint)
		:duration (= ?duration 30)
		:condition (and (at start (robot_at ?r ?from)))
	        :effect (and 
	        		(at start (not (robot_at ?r ?from))) 
				(at end (robot_at ?r ?to)))
)

(:durative-action search
                :parameters (?r - robot ?wp - waypoint ?m - marker)
		:duration (= ?duration 10)
		:condition (and (at start (robot_at ?r ?wp))
				(at start (at_marker ?m ?wp)))
	        :effect (and 
				(at end (found_marker ?m))
				(at end (increase (marker-counter) 1)))
)

)