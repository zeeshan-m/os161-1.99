#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

 static struct semaphore *intersectionSem;
 static struct lock *traffic_lock;
 static volatile bool nwb;
 static volatile bool neb;
 static volatile bool swb;
 static volatile bool seb;
 static struct cv *nw;
 static struct cv *ne;
 static struct cv *sw;
 static struct cv *se;

 bool is_right_turn(Direction, Direction);
 bool is_left_turn(Direction, Direction);
 bool is_straight(Direction, Direction);

 void turn_right(Direction, Direction);
 void turn_left(Direction, Direction);
 void go_straight(Direction, Direction);
 void undo_turn_right(Direction, Direction);
 void undo_turn_left(Direction, Direction);
 void undo_go_straight(Direction, Direction);
 static void print_direction(Direction);
/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
 void
 intersection_sync_init(void)
 {
  intersectionSem = sem_create("intersectionSem",1);
  if (intersectionSem == NULL) {
    panic("could not create intersection semaphore");
  }

  traffic_lock = lock_create("traffic lock");
  if (traffic_lock == NULL) {
    panic("could not create traffic lock");
  }
  
  nw = cv_create("nw cv");
  ne = cv_create("ne cv");
  sw = cv_create("sw cv");
  se = cv_create("se cv");
  nwb = false;
  neb = false;
  swb = false;
  seb = false;
  if (nw == NULL || ne == NULL || sw == NULL || se == NULL) {
    panic ("Could not create intersection cvs");
  }
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
 void
 intersection_sync_cleanup(void)
 {
  KASSERT(intersectionSem != NULL);
  KASSERT(nw != NULL);
  KASSERT(ne != NULL);
  KASSERT(sw != NULL);
  KASSERT(se != NULL);
  KASSERT(traffic_lock != NULL);

  sem_destroy(intersectionSem);
  cv_destroy(nw);
  cv_destroy(ne);
  cv_destroy(sw);
  cv_destroy(se);
  lock_destroy(traffic_lock);
}

bool is_right_turn(Direction origin, Direction destination) {
  bool right_turn = false;
  if (origin == north && destination == west) {
    right_turn = true;
  } else if (origin == west && destination == south) {
    right_turn = true;
  } else if (origin == south && destination == east) {
    right_turn = true;
  } else if (origin == east && destination == north) {
    right_turn = true;
  }
  return right_turn;
}

bool is_left_turn(Direction origin, Direction destination) {
  bool left_turn = false;
  if (origin == north && destination == east) {
    left_turn = true;
  } else if (origin == west && destination == north) {
    left_turn = true;
  } else if (origin == south && destination == west) {
    left_turn = true;
  } else if (origin == east && destination == south) {
    left_turn = true;
  }
  return left_turn;
}

bool is_straight(Direction origin, Direction destination) {
  bool straight = false;
  if (origin == north && destination == south) {
    straight = true;
  } else if (origin == west && destination == east) {
    straight = true;
  } else if (origin == south && destination == north) {
    straight = true;
  } else if (origin == east && destination == west) {
    straight = true;
  }
  return straight;
}

static void print_direction(Direction d) {
  switch (d)
  {
    case north:
    kprintf("N");
    break;
    case east:
    kprintf("E");
    break;
    case south:
    kprintf("S");
    break;
    case west:
    kprintf("W");
    break;
  }
}    

void intersection_before_entry(Direction origin, Direction destination) {
  KASSERT(intersectionSem != NULL);
  KASSERT(nw != NULL);
  KASSERT(ne != NULL);
  KASSERT(sw != NULL);
  KASSERT(se != NULL);
  KASSERT(traffic_lock != NULL);
  lock_acquire(traffic_lock);
  if(is_right_turn(origin, destination)) {
    turn_right(origin, destination);
  } else if (is_left_turn(origin, destination)) {
    turn_left(origin, destination);
  } else if (is_straight(origin, destination)) {
    go_straight(origin, destination);
  }
  lock_release(traffic_lock);
}

void turn_right(Direction origin, Direction destination) {
  kprintf("Right");
  print_direction(origin);
  kprintf("->");
  print_direction(destination);
  kprintf("\n");
  kprintf("Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
  if(origin == south) {
    if(seb) {
      while (seb) {
        kprintf("RIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(se, traffic_lock);
      }
      seb = true;
    } else {
      seb = true;
    }
  } else if(origin == north) {
    if(nwb) {
      while (nwb) {
        kprintf("RIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(nw, traffic_lock);        
      }
      nwb = true;
    } else {
      nwb = true;
    }
  } else if(origin == west) {
    if(swb) {
      while (swb) {
        kprintf("RIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(sw, traffic_lock);        
      }
      swb = true;
    } else {
      swb = true;
    }
  } else if(origin == east) {
    if(neb) {
      while (neb) {
        kprintf("RIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(ne, traffic_lock);
      }
      neb = true;
    } else {
      neb = true;
    }
  }
}

void turn_left(Direction origin, Direction destination) {
  (void)destination;
  kprintf("Left");
  print_direction(origin);
  kprintf("->");
  print_direction(destination);
  kprintf("\n");
  kprintf("Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
  if(origin == south || origin == north) {
    if(nwb && !seb) {
      seb = true;
      while (nwb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(nw, traffic_lock);
      }
      nwb = true;
    } else if (!nwb && seb) {
      nwb = true;
      while (seb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(se, traffic_lock);
      }
      seb = true;
    } else if (nwb && seb) {
      while (nwb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(nw, traffic_lock);
      }
      nwb = true;
      while (seb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(se, traffic_lock);
      }
      seb = true;
    } else {
      nwb = true;
      seb = true;
    }
  } else if(origin == west || origin == east) {
    if(swb && !neb) {
      neb = true;
      while (swb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(sw, traffic_lock);        
      }
      swb = true;
    } else if (!swb && neb) {
      swb = true;
      while(neb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(ne, traffic_lock);        
      }
      neb = true;
    } else if (neb && swb) {
      while (neb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(ne, traffic_lock);
      }
      neb = true;
      while(swb) {
        kprintf("LEFT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(sw, traffic_lock);
      }
      swb = true;
    } else {
      neb = true;
      swb = true;
    }
  }
}

void go_straight(Direction origin, Direction destination) {
  (void)destination;
  kprintf("Straight");
  print_direction(origin);
  kprintf("->");
  print_direction(destination);
  kprintf("\n");
  kprintf("Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
  if(origin == south) {
    bool got_se = false;
    bool got_ne = false;
    if (!seb) {
      seb = true;
      got_se = true;
    }
    if (!neb) {
      neb = true;
      got_ne = true;
    }
    if(!got_ne) {
      while (neb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(ne, traffic_lock);        
      }
      neb = true;
    }
    if(!got_se) {
      while (seb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(se, traffic_lock);        
      }
      seb = true;
    }
  } else if(origin == north) {
    bool got_sw = false;
    bool got_nw = false;
    if (!swb) {
      swb = true;
      got_sw = true;
    }
    if (!nwb) {
      nwb = true;
      got_nw = true;
    }
    if(!got_nw) {
      while (nwb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(nw, traffic_lock);        
      }
      nwb = true;
    }
    if(!got_sw) {
      while (swb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(sw, traffic_lock);        
      }
      swb = true;
    }
  } else if(origin == west) {
    bool got_se = false;
    bool got_sw = false;
    if (!seb) {
      seb = true;
      got_se = true;
    }
    if (!swb) {
      swb = true;
      got_sw = true;
    }
    if(!got_sw) {
      while (swb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(sw, traffic_lock);        
      }
      swb = true;
    }
    if(!got_se) {
      while (seb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(se, traffic_lock);        
      }
      seb = true;
    }
  } else if(origin == east) {
    bool got_ne = false;
    bool got_nw = false;
    if (!neb) {
      neb = true;
      got_ne = true;
    }
    if (!nwb) {
      nwb = true;
      got_nw = true;
    }
    if(!got_nw) {
      while(nwb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(nw, traffic_lock);        
      }
      nwb = true;
    }
    if(!got_ne) {
      while (neb) {
        kprintf("STRAIGHT WAITING Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
        cv_wait(ne, traffic_lock);        
      }
      neb = true;
    }
  }
}

///
void undo_turn_right(Direction origin, Direction destination) {
  (void)destination;
  kprintf("Undo right Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
  if(origin == south) {
    seb = false;
    cv_signal(se, traffic_lock);
  } else if(origin == north) {
    nwb = false;
    cv_signal(nw, traffic_lock);
  } else if(origin == west) {
    swb = false;
    cv_signal(sw, traffic_lock);
  } else if(origin == east) {
    neb = false;
    cv_signal(ne, traffic_lock);
  }
}

void undo_turn_left(Direction origin, Direction destination) {
  (void)destination;
  kprintf("Undo Left Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);
  if(origin == south || origin == north) {
    nwb = false;
    seb = false;
    cv_signal(nw, traffic_lock);
    cv_signal(se, traffic_lock);
  } else if(origin == west || origin == east) {
    neb = false;
    swb = false;
    cv_signal(ne, traffic_lock);
    cv_signal(sw, traffic_lock);
  }
}

void undo_go_straight(Direction origin, Direction destination) {
  (void)destination;
  kprintf("Undo straight Bools: nwb %d, neb %d, swb %d, seb %d \n", nwb, neb, swb, seb);

  if(origin == south) {
    seb = false;
    neb = false;
    cv_signal(ne, traffic_lock);
    cv_signal(se, traffic_lock);
  } else if(origin == north) {
    nwb = false;
    swb = false;
    cv_signal(nw, traffic_lock);
    cv_signal(sw, traffic_lock);
  } else if(origin == west) {
    swb = false;
    seb = false;
    cv_signal(sw, traffic_lock);
    cv_signal(se, traffic_lock);
  } else if(origin == east) {
    nwb = false;
    neb = false;
    cv_signal(nw, traffic_lock);
    cv_signal(ne, traffic_lock);
  }
}

/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

 void
 intersection_after_exit(Direction origin, Direction destination) 
 {
  KASSERT(intersectionSem != NULL);
  KASSERT(nw != NULL);
  KASSERT(ne != NULL);
  KASSERT(sw != NULL);
  KASSERT(se != NULL);
  KASSERT(traffic_lock != NULL);

  lock_acquire(traffic_lock);
  if(is_right_turn(origin, destination)) {
    undo_turn_right(origin, destination);
  } else if (is_left_turn(origin, destination)) {
    undo_turn_left(origin, destination);
  } else if (is_straight(origin, destination)) {
    undo_go_straight(origin, destination);
  }
  lock_release(traffic_lock);


//V(intersectionSem);
}
