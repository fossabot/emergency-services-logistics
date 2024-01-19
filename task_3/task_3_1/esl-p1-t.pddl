(define (problem esl-p1-t) (:domain emergency-services-logistics-temporal)
(:objects 
    b1 b2 b3 b4 b5 - box
    depot l1 l2 - location
    p1 p2 p3 - person
    r1 - robot
    ca - carrier
    s1-ca s2-ca s3-ca s4-ca - slot
    food medicine tool - content
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (depot-at depot)

    (at b1 depot) (at b2 depot) (at b3 depot)(at b4 depot) (at b5 depot)
    (at food depot) (at medicine depot) (at tool depot)
    (at p1 l1) (at p2 l1) (at p3 l2)
    (at r1 depot) (at ca depot)
    (idle r1)

    (empty s1-ca ca) (empty s2-ca ca)
    (empty s3-ca ca) (empty s4-ca ca)
    
    ( = (content-weight medicine) 1)
    ( = (content-weight food ) 2)
    ( = (content-weight tool) 3)

    ( = (carrier-weight ca) 0)
    ( = (box-weight b1) 0)
    ( = (box-weight b2) 0)
    ( = (box-weight b3) 0)
    ( = (box-weight b4) 0)
    ( = (box-weight b5) 0)

    
)

(:goal (and
    (has-content p1 food )
    (has-content p1 medicine)
    (has-content p2 medicine)
    (has-content p3 food)
    
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)

