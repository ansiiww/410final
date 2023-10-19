; This is a comment line
(define (domain kitchen)
  (:requirements :strips :negative-preconditions :disjunctive-preconditions)

  (:types
    drawer
    top
    cabinet 
    box
    
    robot
  )

  (:predicates
    (above ?x ?y)
    (below ?x ?y)
    (left ?x ?y)
    (right ?x ?y)
    
    (in ?b - box ?d - drawer)
    (on ?x -box ?t - top)
    (open ?d - drawer)

    (sugar ?b - box)
    (red ?d - drawer)
    (green ?d - drawer)    
    (blue ?d - drawer)
    (counter ?t - top)

    (occupied ?r - robot)
    (held ?b - box)
    
  )

  (:action gripFromDrawer :parameters (?r - robot ?b - box ?d - drawer)
    :precondition (and
                    (open ?d) (in ?b ?d)
                    (not (occupied ?r)) (not (held ?b)))
    :effect (and 
                (not (in ?b ?d))
                (occupied ?r) (held ?b))
    )

  (:action gripFromTop :parameters (?r - robot ?b - box ?t - top)
    :precondition (and
                    (on ?b ?t)
                    (not (occupied ?r)) (not (held ?b)))
    :effect (and 
                (not (on ?b ?t))
                (occupied ?r) (held ?b))
    )

  (:action dropOnTop :parameters (?r - robot ?b - box ?t - top)
    :precondition (and
                    (not (on ?b ?t))
                    (occupied ?r) (held ?b))
    :effect (and 
                (on ?b ?t)
                (not (occupied ?r)) (not (held ?b)))
    )

  (:action dropInDrawer :parameters (?r - robot ?b - box ?d - drawer)
    :precondition (and
                    (open ?d) (not (in ?b ?d))
                    (occupied ?r) (held ?b))
                    
    :effect (and 
                (in ?b ?d)
                (not (occupied ?r)) (not (held ?b)))
    )

  (:action openDrawer :parameters (?r - robot ?d - drawer)
    :precondition (and 
                    (not (open ?d)) (not (occupied ?r)))
    :effect (and 
                (open ?d) )
    )

  (:action closeDrawer :parameters (?r - robot ?d - drawer)
    :precondition (and 
                    (open ?d) (not (occupied ?r)))
    :effect (and 
                (not (open ?d)))
    ) 

)


