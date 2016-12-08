#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#define MAX_NUM_BLOCKS 64
#define LEFT_CHILD_INDEX(x) ((2 * x))
#define RIGHT_CHILD_INDEX(x) ((2 * x + 1))
#define PARENT_INDEX(x) ((x / 2))
#define SQUARE(x) ((x * x))
#define LEFT 0
#define RIGHT 1
#define TOP 2
#define BOTTOM 3
#define LENGTH 8
#define WIDTH 8


// Neil's hardware code
#define leftPin A2
#define rightPin A5
#define frontPin A0

#define DIR_PIN_LEFT 3
#define STEP_PIN_LEFT 7

#define DIR_PIN_RIGHT 5
#define STEP_PIN_RIGHT 4

bool returnedToPosition = true;
bool finishedTurning = false;
bool turnedLeft = false;
bool turnedRight = false;
bool movedForward = true;

int flaggg = true;

int irPin[3] = {frontPin, leftPin, rightPin};
int i, j;
double distance[3] = {0};

// end neil's constants


struct Coordinate {
  byte row;
  byte col;
};

struct Block {
  struct Coordinate coord;
  bool visited;
  struct Block* prev;
};

struct Priority_Queue {
  struct Block* data[MAX_NUM_BLOCKS];
  byte size;
};

typedef struct Coordinate Coordinate;
typedef struct Block Block;
typedef struct Priority_Queue Priority_Queue;

Coordinate dest;
Block dest_block;
Block curr_block;
Coordinate origin;
Priority_Queue pq;
Coordinate current;
int direction;

Block* grid[LENGTH][WIDTH];

// map of block ids to distances
double distances[MAX_NUM_BLOCKS];


Block* peek_highest_priority(Priority_Queue pq);
Block* pop_highest_priority(Priority_Queue pq);
void pq_add(Priority_Queue pq, Block b);
bool is_empty(Priority_Queue pq);
void sink(Priority_Queue pq, byte ind);
void swim(Priority_Queue pq, byte ind);
void swap(Block blocks[], byte index_one, byte index_two);
double calculate_distance(Coordinate origin, Coordinate dest);
int run_Astar(Priority_Queue pq);
void visit(Priority_Queue pq, Block b);
bool equals(Block b, Block d);
byte get_index(Block b);
double get_distance(Block b);
bool is_inbounds(Coordinate coord);
int get_direction(Block b); // gets the direction of block b with respect to current


/* Returns if the dest block is reachable given the current block */
bool is_reachable(Block dest);

void rotateDeg(float deg, float speed, int dirPin, int stepPin);
double get_IR (uint16_t value);
void readIRValue();

bool is_left_open();
bool is_right_open();
bool is_front_open();
void move_forward_block();
void turn_around();
void turn_left();
void turn_right();

void move_robot(Block current, Block dest); /* Moves robot to dest block from current block */


void setup() {

  // set up Neil's hardware code
  Serial.begin (9600);
  for(i = 0; i < 3; i++) pinMode(irPin[i], INPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(STEP_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(STEP_PIN_RIGHT, OUTPUT);
  // end Neil's setup

  direction = BOTTOM;
  memset(&distances, 1000, MAX_NUM_BLOCKS);

  dest = {.row = 0, .col = 0}; // reset this before solving maze
  dest_block = {dest, 0, NULL};
  origin = {.row = 0, .col = 0}; // origin is always 0, 0
  current = {.row = 0, .col = 0};
  pq = {}; // initialize empty priority queue
  for (int row = 0; row < LENGTH; row++) {
    for (int col = 0; col < WIDTH; col++) {
      if (row == dest.row && col == dest.col) {
        grid[row][col] = &dest_block;
      } else if (row == 0 && col == 0) {
        Block start_block = {origin, 0, NULL};
        grid[row][col] = &start_block;
      } else {
        Coordinate coord = {row, col};
        Block new_block = {coord, 0, NULL};
        grid[row][col] = &new_block;
      }
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

int run_Astar(Priority_Queue pq) {
  Block start_block = *grid[0][0];
  curr_block = start_block;
  pq_add(pq, start_block);

  // distance to start is 0 + heuristic
  distances[get_index(start_block)] = 0 + calculate_distance(start_block.coord, dest);

  // while pq not empty, go to most optimal
  while (!is_empty(pq)) {
    Block* best_block = pop_highest_priority(pq);
    assert(best_block != NULL);
    if (best_block->visited) {
      continue;
    }

    // move robot to best_block;
    move_robot(curr_block, *best_block);
    curr_block = *best_block;

    if (equals(*best_block, dest_block)) {
      return 1;
    } else {
      visit(pq, *best_block);
    }
  }

  return 0;
}

void visit(Priority_Queue pq, Block b) {
  b.visited = true;
  byte curr_row = b.coord.row;
  byte curr_col = b.coord.col;

  // for every neighbor, check if we have a shorter distance
  for (int row_offset = -1; row_offset <= 1; row_offset++) {
    for (int col_offset = -1; col_offset <= 1; col_offset++) {

      Coordinate new_coord = {curr_row + row_offset, curr_col + col_offset};
      // only check four directions, top, bot, left, right, make sure in bounds
      if (row_offset ^ col_offset && is_inbounds(new_coord)) {

        // get the neighbor from the grid, make sure it's reachable and hasn't been seen
        Block neighbor = *grid[curr_row + row_offset][curr_col + col_offset];
        if (is_reachable(neighbor) && !neighbor.visited) {

          // update distances of neighbor
          double oldDist = distances[get_index(neighbor)];
          double newDist = distances[get_index(b)] + 1 + calculate_distance(neighbor.coord, dest);
          // add to queue
          if (newDist < oldDist) {
            neighbor.prev = &b;
            distances[get_index(neighbor)] = newDist;
            pq_add(pq, neighbor);
          }
        }
      }
    }
  }
}

// determines if a neighboring block is reachable
bool is_reachable(Block b) {
  int dir_block_b = get_direction(b);
  // see where the block is in respect to current block
  switch (dir_block_b) {
    case TOP:
      if (direction == TOP) {
        return is_front_open();
      } else if (direction == LEFT) {
        return is_right_open();
      } else if (direction == RIGHT) {
        return is_left_open();
      } else {
        return true;
      }
    case LEFT:
      if (direction == TOP) {
        return is_left_open();
      } else if (direction == LEFT) {
        return is_front_open();
      } else if (direction == RIGHT) {
        return true;
      } else {
        return is_right_open();
      }
    case RIGHT:
      if (direction == TOP) {
        return is_right_open();
      } else if (direction == LEFT) {
        return true;
      } else if (direction == RIGHT) {
        return is_front_open();
      } else {
        return is_left_open();
      }
    case BOTTOM:
      if (direction == TOP) {
        return true;
      } else if (direction == LEFT) {
        return is_left_open();
      } else if (direction == RIGHT) {
        return is_right_open();
      } else {
        return is_front_open();
      }
  }

  return false;
}

void move_robot(Block curr, Block dest) {
  if (equals(curr, dest)) return;
  // use previous fields to find a path from curr to dest

  // move robot using discrete movements
}

bool is_inbounds(Coordinate coord) {
  return (coord.row >= 0 && coord.row < LENGTH && coord.col >= 0 && coord.col < WIDTH);
}

byte get_index(Block b) {
  short row = b.coord.row;
  short col = b.coord.col;
  return col + (row * 16);
}

bool equals(Block b, Block d) {
  return b.coord.row == d.coord.row && b.coord.col == d.coord.col;
}

double square(double num) {
  return num * num;
}

double get_distance(Block b) {
  return distances[get_index(b)];
}

int get_direction(Block b) {
  byte curr_row = curr_block.coord.row;
  byte curr_col = curr_block.coord.col;
  byte b_row = b.coord.row;
  byte b_col = b.coord.col;

  if (b_col == curr_col) {
    if (curr_row < b_row) {
      return BOTTOM;
    } else {
      return TOP;
    }
  }

  if (b_row == curr_row) {
    if (b_col < curr_col) {
      return LEFT;
    } else {
      return RIGHT;
    }
  }

  return TOP;
}

double calculate_distance(Coordinate origin, Coordinate dest) {
  double origin_y = (double) origin.row;
  double origin_x = (double) origin.col;
  double dest_y = (double) dest.row;
  double dest_x = (double) dest.col;
  return sqrt(SQUARE(origin_y - dest_y) + SQUARE(origin_x - dest_x));
}

bool is_empty(Priority_Queue pq) {
  return !pq.size;
}

Block* peek_highest_priority(Priority_Queue pq) {
  if (pq.size == 0) {
    return NULL;
  } else {
    return pq.data[0];
  }
}

Block* pop_highest_priority(Priority_Queue pq) {
  if (pq.size == 0) {
    return NULL;
  } else {
    Block* to_return = pq.data[0];
    pq.data[0] = pq.data[pq.size - 1];
    sink(pq, 0);
    return to_return;
  }
}

void pq_add(Priority_Queue pq, Block b) {
  if (pq.size >= MAX_NUM_BLOCKS) return;
  pq.data[pq.size] = &b;
  swim(pq, pq.size);
  pq.size++;
}

void swap(Priority_Queue pq, short index_one, short index_two) {
  Block* temp = pq.data[index_one];
  pq.data[index_one] = pq.data[index_two];
  pq.data[index_two] = temp;
}

void sink(Priority_Queue pq, byte index) {
  byte curr_index = index;

  // while we still have children in pq
  while (LEFT_CHILD_INDEX(curr_index) < pq.size) {
    Block* curr_block = pq.data[curr_index];
    byte smaller_index = LEFT_CHILD_INDEX(curr_index);
    byte right_idx = (RIGHT_CHILD_INDEX(curr_index) < pq.size) ? RIGHT_CHILD_INDEX(curr_index) : -1;
    Block* smaller_block = pq.data[smaller_index];
    Block* right_block = NULL;
    if (right_idx < pq.size) {
      right_block = pq.data[right_idx];
    }

    if (right_block != NULL && get_distance(*right_block) < get_distance(*smaller_block)) {
      smaller_block = right_block;
    }

    if (get_distance(*smaller_block) < get_distance(*curr_block)) {
      swap(pq, smaller_index, curr_index);
      curr_index = smaller_index;
    } else {
      return;
    }
  }

  return;
}

void swim(Priority_Queue pq, byte index) {
  byte curr_index = index;

  while (curr_index > 0) {
    Block* curr_block = pq.data[curr_index];
    byte parent_idx = PARENT_INDEX(curr_index);
    Block* parent_block = NULL;
    if (parent_idx >= 0) {
      parent_block = pq.data[parent_idx];
    }

    assert(parent_block);
    
    if (get_distance(*parent_block) > get_distance(*curr_block)) {
      swap(pq, parent_idx, curr_index);
      curr_index = parent_idx;
    } else {
      return;
    }
  }

  return;
}

// begin Neil's hardware code

void rotateDeg(float deg, float speed, int dirPin, int stepPin){ 
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0)? HIGH:LOW;
  digitalWrite(dirPin,dir); 

  int steps = abs(deg)*(1/0.225);
  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){ 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(usDelay); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(usDelay); 
  }
}

double get_IR (uint16_t value) {
  if (value < 16)  value = 16;
  return 2076.0 / (value - 11.0);
}

void readIRValue() {
  float readVal[3] = {0};
  int nSamples = 3;

  for(i = 0; i < nSamples; i++){
    for(j = 0; j < 3; j++){
      readVal[j] = (float)(analogRead(irPin[j]));
      distance[j] = get_IR(readVal[j]);
    }
  }
}

bool is_front_open() {
  return distance[0] > 10;
}

bool is_left_open() {
  return distance[1] > 10;
}

bool is_right_open() {  
  return distance[2] > 10;
}

void turn_left() {
  for (int i = 0; i < 180; i++) {
    rotateDeg(-1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(-1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
}

void turn_right() {
  for (int i = 0; i < 180; i++) {
    rotateDeg(1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
    }
}


void move_forward_block() {
  for (int i = 0; i < 360; i++) {
    rotateDeg(-1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
    }
}

void turn_around() {
  for (int i = 0; i < 360; i++) {
    rotateDeg(1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
    }
}


