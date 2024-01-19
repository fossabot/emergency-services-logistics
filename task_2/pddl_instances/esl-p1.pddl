(define (problem esl-istance-one)

  (:domain emergency-services-logistics)
  (:objects
       r1 - robot
       p1 p2 p3 - person
       b1 b2 b3 b4 b5 - box
       depot l1 l2 - location
       ca - carrier
       s1-ca s2-ca s3-ca s4-ca - slot
       food - content
       medicine - content
  )

  (:init
  	(depot-at depot)
  	(at r1 depot) (at ca depot)
  	(at b1 depot) (at b3 depot) (at b4 depot) (at b5 depot)
  	(at food depot) (at medicine depot)
  	(at p1 l1) (at p2 l1) (at p3 l2)
  	(empty s1-ca ca) (empty s2-ca ca) (empty s3-ca ca) (empty s4-ca ca)
  )

;; The task is to provide people with the content they need
    (:goal
        (and
            (has-content p1 food)
            (has-content p1 medicine)
            (has-content p2 medicine)
            (has-content p3 food)
        )
    )
)

