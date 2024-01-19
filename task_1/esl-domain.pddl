(define (domain emergency-services-logistics)

	(:requirements

		:strips ; actions have only positive preconditions and deterministic effects
        :typing ; restricts objects and actions to certain types
        :conditional-effects
        :equality
        :existential-preconditions
        :adl

	)

	(:types
		slot  ; * A space in a dimensionable object
		box   ; * A box that can be filled with content
		carrier ; * A carrier that can be used to transport boxes
		robot   ; * A robot that can move around and interact with boxes, carrier and contenents
		location  ; * A place
		content   ; * A type of object that can be placed in a box or given to a person
		person ; * A person who can receive content
	)



	(:predicates

		(at ?o - object ?l - location) ;  object ?o is at location ?l
		(depot-at ?l - location) ; A depot is present at location ?l


        ;; box
		(full ?b - box) ; A box ?b that is full.
		(already-taken ?b) ; box ?b it is not available
		(has-inside ?b - box ?elem - content ) ; box ?b has content ?elem
		(on-carrier ?b - box ?c - carrier) ;  box ?b is on carrier ?c


		(empty ?s - slot ?c - carrier) ; A slot ?s of a carrier that is empty.

        ;; person
		(has-content ?p - person ?elem - content ) ; person ?p has content ?elem
		(satisfied-with-at-least-one ?p - person ?elem1 - content ?elem2 - content) ;  person ?p is satisfied with at least one of the objects ?elem1 or ?elem2
		; allows to handle the 'or' in the goal


		(is-holding ?r -robot ?c - carrier) ; robot ?r is holding carrier ?c
	)



	(:action fill-box
		:parameters ( ?r - robot ?b - box ?elem - content ?l - location )
		:precondition (and
		    (not (full ?b)) (depot-at ?l)
		 	(at ?r ?l) (at ?elem ?l) (at ?b ?l)
		)
		:effect (and (full ?b) (has-inside ?b ?elem))
	)

	(:action unfill-box
	    :parameters (?r - robot ?b - box ?elem - content ?l - location)
	    :precondition (and
	        (full ?b) (has-inside ?b ?elem)
	        (depot-at ?l) (at ?r ?l) (at ?b ?l)
	    )
	    :effect (and (not (full ?b)) (not (has-inside ?b ?elem)))
	)

	(
:action give-content
		:parameters (?r - robot ?p - person ?elem - content ?b - box   ?l - location
        	)
		:precondition (and (at ?p ?l) (at ?b ?l) (at ?r ?l)
			(has-inside ?b ?elem) (not (has-content ?p ?elem))
        	)
		:effect (and
		    (not (has-inside ?b ?elem))
            (not (full ?b)) (has-content ?p ?elem)
        	)

	)
	(:action satisfied-with-at-least-one
		:parameters ( ?p - person ?elem1 - content ?elem2 - content)
		:precondition (and
		    (not (satisfied-with-at-least-one ?p ?elem1 ?elem2))
		    (or
			    (has-content ?p ?elem1) (has-content ?p ?elem2)
        	))
		:effect (satisfied-with-at-least-one ?p ?elem1 ?elem2)

	)

	(:action hold-carrier
		:parameters (?r - robot ?c - carrier ?l - location)
		:precondition (and (at ?r ?l) (at ?c ?l)
		    (not
		        (exists (?x - robot)
					(is-holding ?x ?c)
				)
			)
        )
		:effect (is-holding ?r ?c)

	)

	(:action release-carrier
		:parameters (?r - robot ?c - carrier ?l - location)
		:precondition (and (is-holding ?r ?c) (depot-at ?l)
		    (not
            	(exists (?b - box)
            	    (on-carrier ?b ?c)
            	)
            )
		)
		:effect (and (not (is-holding ?r ?c)) (at ?c ?l))
	)

	(:action load-carrier
		:parameters (?r - robot ?b -box ?c - carrier ?s - slot ?l - location)
		:precondition (and
		    (at ?b ?l) (at ?r ?l) (at ?c ?l)
			(not (already-taken ?b))
			(empty ?s ?c)

		)

		:effect (and (on-carrier ?b ?c) (already-taken ?b)
		    (not (empty ?s ?c))
		)

	)

	(:action unload-carrier
        		:parameters (?r - robot ?b - box ?c - carrier ?s - slot ?l - location)
        		:precondition (and
        		    (is-holding ?r ?c)
        		    (at ?r ?l) (at ?c ?l) (depot-at ?l)
        		    (on-carrier ?b ?c) (not (full ?b))
        		    (not (empty ?s ?c) )
                )
        		:effect (and (not (on-carrier ?b ?c)) (at ?b ?l)
        			(empty ?s ?c)
        		)

    )

	(:action move
		:parameters (?r - robot ?from ?to - location)
		:precondition (and (at ?r ?from) (not (at ?r ?to))
				(not
					(exists (?x - carrier)
						(is-holding ?r ?x)
					)
				))
		:effect (and (not (at ?r ?from)) (at ?r ?to))
	)

	(:action move-carrier
		:parameters (?r - robot ?from ?to -location ?c - carrier)
		:precondition (and (at ?r ?from) (not(at ?r ?to)) (at ?c ?from) (is-holding ?r ?c))
		:effect (and (not (at ?r ?from)) (not (at ?c ?from))
            (at ?r ?to) (at ?c ?to)
            (forall (?b - box)
                (when (on-carrier ?b ?c)
                (and (not (at ?b ?from)) (at ?b ?to)
                ))
            )
		)
	)


)