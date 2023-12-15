(define (problem pb1)
  (:domain kitchen)
  (:objects
    d1 d2 d3 - drawer
    t1 t2 t3 - top
    c1 - cabinet
    b1 b2 - box

    r1 - robot
  )
  (:init
    (sugar b1) 
    (red d1)
    (green d2)
    (blue d3)
    (counter t1)

    (on b1 t2) ;sugar box on stove
    (on b2 t1) ;spam on countertop
  )

  (:goal 
    (and 
      (on b1 t3) 
      (in b2 d1)
    )
  )
)