(define (problem esl-istance-two)

    (:domain emergency-services-logistics)


  (:objects


   p1 p2 p3 p4 p5 p6 - person
   b1 b2 b3 - box
   depot l1 l2 l3 l4 l5 - location
   ca cb - carrier
   s1-ca s2-ca s1-cb s2-cb - slot
   food - content
   medicine - content
   tool - content
   r1 r2 - robot

   )



  (:init


  	(depot-at depot)

    (at r1 depot)  (at r2 depot);robot one and robot two are at the same place (Depot)
    (at ca depot) (at cb depot) ;carrier one and carrier two are at the same place (Depot)
  	(at b1 depot) (at b2 depot) (at b3 depot) ;box one box 2 and box 3 are at the same place (Depot)
  	(at food depot) (at medicine depot) (at tool depot) ;food medicine and tools are at the same place (Depot)
  	(at p1 l1) (at p2 l1) (at p3 l2) (at p4 l3) (at p5 l4) (at p6 l5) ; person one and person two are at location one. Other people are at different place.

    (empty s1-ca ca) (empty s2-ca ca) (empty s1-cb cb) (empty s2-cb cb) ; In this case carrier one and also carrier two have two slot.

  )



  (:goal

    (and

        (satisfied-with-at-least-one p1 food tool)
        (has-content p2 medicine)
    	(has-content p3 medicine)

    	(has-content p4 medicine)
    	(has-content p4 food)

    	(has-content p5 food)
    	(has-content p5 medicine)
    	(has-content p5 tool)

    	(has-content p6 food)
        (has-content p6 medicine)
    	(has-content p6 tool)
    )

   )



)

