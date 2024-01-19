(define (domain emergency-services-logistics-temporal)

    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:types
        robot
        slot
        box
        carrier
        location
        content
        person - object
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates

        (at ?o - object ?l - location) ;  object ?o is at location ?l

        (depot-at ?l - location) ; A depot is present at location ?l

        (full ?b - box) ; A box ?b that is full.

        (empty ?s - slot ?c - carrier) ; A slot ?s of a carrier that is empty.

        (on-carrier ?b - box ?c - carrier) ;  box ?b is on carrier ?c

        (already-taken ?b - box) ; box ?b it is not available

        (has-content ?p - person ?elem - content) ; person ?p has content ?elem

        (satisfied-with-at-least-one ?p - person ?elem1 - content ?elem2 - content)

        (has-inside ?b - box ?elem - content); box ?b has content ?elem

        (is-holding ?r -robot ?c - carrier) ; robot ?r is holding carrier ?c

        (is-taken ?c - carrier); carrier ?c it is not available

        (hands-free ?r - robot) ; robot ?r is not holding a carrier

        (idle ?r - robot) ; robot ?r is idle

    );; end Predicates ;;;;;;;;;;;;;;;;;;;;

    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:functions

        (content-weight ?elem - content)

        (box-weight ?b - box)

        (carrier-weight ?c - carrier)

    );; end Functions ;;;;;;;;;;;;;;;;;;;;

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:durative-action move

        :parameters (?r - robot ?from ?to - location)

        :duration (= ?duration 1)

        :condition (and

            (at start

                (and

                    (idle ?r)

                    (at ?r ?from)

                    (not (at ?r ?to))

                )

            )

            (over all

                (hands-free ?r)

            )

        )

        :effect (and

            (at start (and

                    (not (at ?r ?from))

                    (not (idle ?r))

                )

            )

            (at end (and

                    (at ?r ?to)

                    (idle ?r)

                )

            )

        )

    )

    ;;Action Fill-box;;

    (:durative-action fill-box

        :parameters ( ?r - robot ?b - box ?elem - content ?l - location)

        :duration (= ?duration (content-weight ?elem))

        :condition (and

            (at start

                (and

                    (not (full ?b))

                    (idle ?r)

                )

            )

            (over all

                (and

                    (depot-at ?l) (at ?r ?l) (at ?elem ?l)

                    (at ?b ?l)

                )

            )

        )

        :effect (and

            (at start

                (and

                    (full ?b)

                    (not (idle ?r))

                )

            )

            (at end (increase (box-weight ?b) (content-weight ?elem)))

            (at end

                (and

                    (idle ?r)

                    (has-inside ?b ?elem)

                )

            )

        )

    );; end fill-box;;;;

    (:durative-action unfill-box

        :parameters (?r - robot ?b - box ?elem - content ?l - location)

        :duration (= ?duration 2)

        :condition (and

            (at start

                (and

                    (idle ?r)

                    (full ?b)

                    (has-inside ?b ?elem)

                )

            )

            (over all

                (and

                    (depot-at ?l) (at ?r ?l) (at ?elem ?l)

                    (at ?b ?l)

                )

            )

        )

        :effect (and

            (at start

                (not(idle ?r))

            )

            (at end (decrease (box-weight ?b) (content-weight ?elem)))

            (at end

                (and

                    (idle ?r)

                    (not (full ?b)) (not (has-inside ?b ?elem))

                )

            )

        )

    )

    (:durative-action give-content

        :parameters ( ?r - robot ?p - person ?b - box ?elem - content ?l - location ?c - carrier)

        :duration (= ?duration 2)

        :condition (and

            (at start

                (and

                    (idle ?r)

                    (has-inside ?b ?elem)

                    (not (has-content ?p ?elem))

                )

            )

            (over all

                (and

                    (at ?p ?l)

                    (at ?r ?l) (at ?c ?l)

                    (on-carrier ?b ?c)

                )

            )

        )

        :effect (and

            (at start

                (and (not (has-inside ?b ?elem)) (not (full ?b)) (not(idle ?r)))

            )

            (at end (decrease

                    (carrier-weight ?c)

                    (content-weight ?elem)))

            (at end (decrease (box-weight ?b) (content-weight ?elem)))

            (at end

                (and

                    (idle ?r)

                    (has-content ?p ?elem)

                )

            )

        )

    )

    (:durative-action satisfied-with-at-least-one

        :parameters ( ?p - person ?elem1 ?elem2 - content)

        :duration (= ?duration 0)

        :condition (and

            (at start

                (not (satisfied-with-at-least-one ?p ?elem1 ?elem2))

            )

            (over all

                (or (has-content ?p ?elem1) (has-content ?p ?elem2))

            )

        )

        :effect (at end

            (satisfied-with-at-least-one ?p ?elem1 ?elem2)

        )

    )

    (:durative-action hold-carrier

        :parameters (?r - robot ?c - carrier ?l - location)

        :duration (= ?duration 1)

        :condition (and

            (at start (and (idle ?r) (not (is-taken ?c))))

                (over all

                    (and
                        (at ?r ?l) (at ?c ?l)
                    )

                )

            

        )

        :effect (and

            (at start (and (not (idle ?r)) (is-taken ?c)))

            (at end (and

                (is-holding ?r ?c)

                (idle ?r)

                )

            )

            )

        )

    

    (:durative-action release-carrier

        :parameters (?r - robot ?c - carrier ?l - location)

        :duration (= ?duration 1)

        :condition (and

            (at start (idle ?r))

            (over all

                (and (is-holding ?r ?c) (depot-at ?l))

            )

        )

        :effect (and

            (at start (not (idle ?r)))

            (at end

                (and

                    (not (is-holding ?r ?c))

                    (at ?c ?l)

                    (idle ?r)

                )

            )

        )

    )

    ; * da moltiplicare per il peso della cassa

    (:durative-action load-carrier

        :parameters (?r - robot ?b - box ?c - carrier ?s - slot ?l - location)

        :duration (= ?duration 2)

        :condition (and

            (at start

                (and

                    (not (already-taken ?b)) (empty ?s ?c) (idle ?r) (at ?b ?l)

                )

            )

            (over all

                (and

                    (at ?r ?l) (at ?c ?l) (depot-at ?l)

                )

            )

        )

        :effect (and

            (at start

                (and

                    (already-taken ?b) (not (empty ?s ?c)) (not (idle ?r)) (not (at ?b ?l))

                )

            )

            (at end

                (and

                    (on-carrier ?b ?c)

                    (idle ?r)

                )

            )

            (at end (increase (carrier-weight ?c) (box-weight ?b)))

        )

    )

    (:durative-action unload-carrier

        :parameters (?r - robot ?b - box ?c - carrier ?s - slot ?l - location)

        :duration (= ?duration 2)

        :condition (and

            (at start

                (and

                    (on-carrier ?b ?c) (not (empty ?s ?c)) (idle ?r)

                )

            )

            (over all

                (and

                    (at ?r ?l) (at ?c ?l) (depot-at ?l)

                    (not (full ?b))

                    (is-holding ?r ?c)

                )

            )

        )

        :effect (and

            (at start (not(idle ?r)))

            (at end (decrease (carrier-weight ?c) (box-weight ?b)))

            (at end

                (and

                    (idle ?r)

                    (not (on-carrier ?b ?c)) (empty ?s ?c)

                    (at ?b ?l)

                )

            )

        )

    )

    (:durative-action move-carrier

        :parameters (?r - robot ?from ?to - location ?c - carrier)

        :duration (= ?duration (* (carrier-weight ?c) 2))

        :condition (and

            (at start

                (and (at ?r ?from) (not (at ?r ?to)) (idle ?r))

            )

            (over all

                (is-holding ?r ?c)

            )

        )

        :effect (and

            (at start

                (and (not (at ?r ?from))

                    (not (at ?c ?from))

                    (not (idle ?r))

                )

            )

            (at end

                (and (at ?r ?to) (at ?c ?to) (idle ?r))

            )

        )

    )

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
