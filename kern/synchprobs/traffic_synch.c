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

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
 static struct lock *traffic_entering;
 static struct lock *traffic_exiting;

 static struct lock *north_right;
 static struct lock *north_straight;
 static struct lock *north_left;

 static struct lock *south_right;
 static struct lock *south_straight;
 static struct lock *south_left;

 static struct lock *east_right;
 static struct lock *east_straight;
 static struct lock *east_left;

 static struct lock *west_right;
 static struct lock *west_straight;
 static struct lock *west_left;

 bool is_right_turn(Direction, Direction);
 bool is_left_turn(Direction, Direction);
 bool is_straight(Direction, Direction);

 void turn_right(Direction, Direction);
 void turn_left(Direction, Direction);
 void go_straight(Direction, Direction);

 void undo_turn_right(Direction, Direction);
 void undo_turn_left(Direction, Direction);
 void undo_go_straight(Direction, Direction);


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
  traffic_exiting = lock_create("traffic_exiting");
  if(traffic_exiting == NULL) {
    panic ("Could not create traffic exiting lock");
  }
  traffic_entering = lock_create("traffic_entering");
  if(traffic_entering == NULL) {
    panic("Could not create traffic entering lock");
  }
  north_right = lock_create("north_right");
  north_straight = lock_create("north_straight");
  north_left = lock_create("north_left");

  south_right = lock_create("south_right");
  south_straight = lock_create("south_straight");
  south_left = lock_create("south_left");

  east_right = lock_create("east_right");
  east_straight = lock_create("east_straight");
  east_left = lock_create("east_left");

  west_right = lock_create("west_right");
  west_straight = lock_create("west_straight");
  west_left = lock_create("west_left");

  if (north_right == NULL || north_straight == NULL || north_left == NULL || south_right == NULL || south_straight == NULL || south_left == NULL || east_right == NULL || east_straight == NULL || east_left == NULL || west_right == NULL || west_straight == NULL || west_left == NULL) {
    panic("could not create intersection semaphore");
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
  KASSERT(traffic_exiting != NULL);
  KASSERT(traffic_entering != NULL);
  KASSERT(north_right != NULL);
  KASSERT(north_straight != NULL);
  KASSERT(north_left != NULL);
  KASSERT(south_right != NULL);
  KASSERT(south_straight != NULL);
  KASSERT(south_left != NULL);
  KASSERT(east_right != NULL);
  KASSERT(east_straight != NULL);
  KASSERT(east_left != NULL);
  KASSERT(west_right != NULL);
  KASSERT(west_straight != NULL);
  KASSERT(west_left != NULL);

  lock_destroy(traffic_entering);
  lock_destroy(traffic_exiting); 

  lock_destroy(north_right); 
  lock_destroy(north_straight); 
  lock_destroy(north_left); 

  lock_destroy(south_right); 
  lock_destroy(south_straight); 
  lock_destroy(south_left); 

  lock_destroy(east_right); 
  lock_destroy(east_straight); 
  lock_destroy(east_left); 

  lock_destroy(west_right); 
  lock_destroy(west_straight); 
  lock_destroy(west_left); 
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

 void
 intersection_before_entry(Direction origin, Direction destination) 
 {
  KASSERT(traffic_entering != NULL);
  KASSERT(north_right != NULL);
  KASSERT(north_straight != NULL);
  KASSERT(north_left != NULL);
  KASSERT(south_right != NULL);
  KASSERT(south_straight != NULL);
  KASSERT(south_left != NULL);
  KASSERT(east_right != NULL);
  KASSERT(east_straight != NULL);
  KASSERT(east_left != NULL);
  KASSERT(west_right != NULL);
  KASSERT(west_straight != NULL);
  KASSERT(west_left != NULL);
  lock_acquire(traffic_entering);
  //kprintf("Car Entering: %d->%d\n", origin, destination);
  if(is_right_turn(origin, destination)) {
    turn_right(origin, destination);
  } else if (is_left_turn(origin, destination)) {
    turn_left(origin, destination);
  } else if (is_straight(origin, destination)) {
    go_straight(origin, destination);
  }
  lock_release(traffic_entering);
  //kprintf("Car Entered: %d->%d\n", origin, destination);
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
  KASSERT(traffic_exiting != NULL);
  KASSERT(north_right != NULL);
  KASSERT(north_straight != NULL);
  KASSERT(north_left != NULL);
  KASSERT(south_right != NULL);
  KASSERT(south_straight != NULL);
  KASSERT(south_left != NULL);
  KASSERT(east_right != NULL);
  KASSERT(east_straight != NULL);
  KASSERT(east_left != NULL);
  KASSERT(west_right != NULL);
  KASSERT(west_straight != NULL);
  KASSERT(west_left != NULL);
  
  //kprintf("Car Exiting: %d->%d\n", origin, destination);
  lock_acquire(traffic_exiting);
  //kprintf("Car Exiting: %d->%d\n", origin, destination);

  if(is_right_turn(origin, destination)) {
    undo_turn_right(origin, destination);
  } else if (is_left_turn(origin, destination)) {
    undo_turn_left(origin, destination);
  } else if (is_straight(origin, destination)) {
    undo_go_straight(origin, destination);
  }
  lock_release(traffic_exiting);
}



void turn_right(Direction origin, Direction destination) {
  (void)destination;
  //kprintf("Getting right locks");
  if(origin == north) {
    lock_acquire(east_straight);
    lock_acquire(south_left);

    lock_acquire(north_right);

    lock_release(east_straight);
    lock_release(south_left);
  } else if (origin == south) {
    lock_acquire(west_straight);
    lock_acquire(north_left);

    lock_acquire(south_right);

    lock_release(west_straight);
    lock_release(north_left);
  } else if (origin == west) {
    lock_acquire(north_straight);
    lock_acquire(east_left);

    lock_acquire(west_right);

    lock_release(north_straight);
    lock_release(east_left);
  } else if (origin == east) {
    lock_acquire(south_straight);
    lock_acquire(west_left);

    lock_acquire(east_right);

    lock_release(south_straight);
    lock_release(west_left);
  }
  //kprintf("Got right locks");
}

void turn_left(Direction origin, Direction destination) {
  (void)destination;
  //kprintf("Getting left locks\n");
  if(origin == north) {
    lock_acquire(south_right);
    lock_acquire(south_straight);
    lock_acquire(south_left);

    lock_acquire(west_straight);
    lock_acquire(west_left);

    lock_acquire(east_straight);
    lock_acquire(east_left);

    lock_acquire(north_left);

    lock_release(south_right);
    lock_release(south_straight);
    lock_release(south_left);

    lock_release(west_straight);
    lock_release(west_left);

    lock_release(east_straight);
    lock_release(east_left);
  } else if (origin == south) {
    lock_acquire(north_right);
    lock_acquire(north_straight);
    lock_acquire(north_left);

    lock_acquire(west_straight);
    lock_acquire(west_left);

    lock_acquire(east_straight);
    lock_acquire(east_left);

    lock_acquire(south_left);

    lock_release(north_right);
    lock_release(north_straight);
    lock_release(north_left);

    lock_release(west_straight);
    lock_release(west_left);

    lock_release(east_straight);
    lock_release(east_left);
  } else if (origin == west) {
    lock_acquire(east_right);
    lock_acquire(east_straight);
    lock_acquire(east_left);

    lock_acquire(north_straight);
    lock_acquire(north_left);

    lock_acquire(south_straight);
    lock_acquire(south_left);

    lock_acquire(west_left);

    lock_release(east_right);
    lock_release(east_straight);
    lock_release(east_left);

    lock_release(north_straight);
    lock_release(north_left);

    lock_release(south_straight);
    lock_release(south_left);
  } else if (origin == east) {
    lock_acquire(west_right);
    lock_acquire(west_straight);
    lock_acquire(west_left);

    lock_acquire(north_straight);
    lock_acquire(north_left);

    lock_acquire(south_straight);
    lock_acquire(south_left);

    lock_acquire(east_left);

    lock_release(west_right);
    lock_release(west_straight);
    lock_release(west_left);

    lock_release(north_straight);
    lock_release(north_left);

    lock_release(south_straight);
    lock_release(south_left);
  }
  //kprintf("Got left locks\n");
}

void go_straight(Direction origin, Direction destination) {
  (void)destination;
  //kprintf("Getting straight locks\n");
  if(origin == north) {
    lock_acquire(west_straight);
    lock_acquire(west_left);
    lock_acquire(west_right);

    lock_acquire(south_left);

    lock_acquire(east_straight);
    lock_acquire(east_left);

    lock_acquire(north_straight);

    lock_release(west_straight);
    lock_release(west_left);
    lock_release(west_right);

    lock_release(south_left);

    lock_release(east_straight);
    lock_release(east_left);
  } else if (origin == south) {
    lock_acquire(east_straight);
    lock_acquire(east_left);
    lock_acquire(east_right);

    lock_acquire(north_left);

    lock_acquire(west_straight);
    lock_acquire(west_left);

    lock_acquire(south_straight);

    lock_release(east_straight);
    lock_release(east_left);
    lock_release(east_right);

    lock_release(north_left);

    lock_release(west_straight);
    lock_release(west_left);
  } else if (origin == west) {
    lock_acquire(south_straight);
    lock_acquire(south_left);
    lock_acquire(south_right);

    lock_acquire(east_left);

    lock_acquire(north_straight);
    lock_acquire(north_left);

    lock_acquire(west_straight);

    lock_release(south_straight);
    lock_release(south_left);
    lock_release(south_right);

    lock_release(east_left);

    lock_release(north_straight);
    lock_release(north_left);
  } else if (origin == east) {
    lock_acquire(north_straight);
    lock_acquire(north_left);
    lock_acquire(north_right);

    lock_acquire(west_left);

    lock_acquire(south_straight);
    lock_acquire(south_left);

    lock_acquire(east_straight);

    lock_release(north_straight);
    lock_release(north_left);
    lock_release(north_right);

    lock_release(west_left);

    lock_release(south_straight);
    lock_release(south_left);
  }
  //kprintf("Got straight locks\n");
}

void undo_turn_right(Direction origin, Direction destination) {
  (void)destination;
  if(origin == north) {
    lock_release(north_right);
  } else if (origin == south) {
    lock_release(south_right);
  } else if (origin == west) {
    lock_release(west_right);
  } else if (origin == east) {
    lock_release(east_right);
  }
}

void undo_turn_left(Direction origin, Direction destination) {
  (void)destination;
  if(origin == north) {
    lock_release(north_left);
  } else if (origin == south) {
    lock_release(south_left);
  } else if (origin == west) {
    lock_release(west_left);
  } else if (origin == east) {
    lock_release(east_left);
  }
}

void undo_go_straight(Direction origin, Direction destination) {
  (void)destination;
  if(origin == north) {
    lock_release(north_straight);
  } else if (origin == south) {
    lock_release(south_straight);
  } else if (origin == west) {
    lock_release(west_straight);
  } else if (origin == east) {
    lock_release(east_straight);
  }

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
