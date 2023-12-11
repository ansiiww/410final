; This is a comment line
(define (domain kitchen)
  (:requirements :strips :negative-preconditions :typing) 
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
    (on ?b - box ?t - top)
    (open ?d - drawer)

    (sugar ?b - box)
    (red ?d - drawer)
    (green ?d - drawer)    
    (blue ?d - drawer)
    (counter ?t - top)

    (occupied ?r - robot)
    (held ?b - box)
    
  )

  (:action grip_From_Drawer :parameters (?r - robot ?b - box ?d - drawer)
    :precondition (and
                    (open ?d) (in ?b ?d)
                    (not (occupied ?r)) (not (held ?b)))
    :effect (and 
                (not (in ?b ?d))
                (occupied ?r) (held ?b))
    )

  (:action grip_From_Top :parameters (?r - robot ?b - box ?t - top)
    :precondition (and
                    (on ?b ?t)
                    (not (occupied ?r)) (not (held ?b)))
    :effect (and 
                (not (on ?b ?t))
                (occupied ?r) (held ?b))
    )

  (:action drop_On_Top :parameters (?r - robot ?b - box ?t - top)
    :precondition (and
                    (not (on ?b ?t))
                    (occupied ?r) (held ?b))
    :effect (and 
                (on ?b ?t)
                (not (occupied ?r)) (not (held ?b)))
    )

  (:action drop_In_Drawer :parameters (?r - robot ?b - box ?d - drawer)
    :precondition (and
                    (open ?d) (not (in ?b ?d))
                    (occupied ?r) (held ?b))
                    
    :effect (and 
                (in ?b ?d)
                (not (occupied ?r)) (not (held ?b)))
    )

  (:action open_Drawer :parameters (?r - robot ?d - drawer)
    :precondition (and 
                    (not (open ?d)) (not (occupied ?r)))
    :effect (and 
                (open ?d) )
    )

  (:action close_Drawer :parameters (?r - robot ?d - drawer)
    :precondition (and 
                    (open ?d) (not (occupied ?r)))
    :effect (and 
                (not (open ?d)))
    ) 

)


