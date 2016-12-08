#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <QueueList.h>
#include <StackList.h>

#define MAX_NUM_BLOCKS 64
#define LEFT 0
#define RIGHT 1
#define TOP 2
#define BOTTOM 3
#define LENGTH 8
#define WIDTH 8
#define DEST_ROW 2
#define DEST_COL 2

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
  int row;
  int col;
};

struct Block {
  struct Coordinate coord;
  bool visited;
  struct Block* prev;
  int distance;
};

typedef struct Coordinate Coordinate;
typedef struct Block Block;

Block dest_block;
Block curr_block;
int direction;
int curr_distance;
bool run_once = true;
Block grid[LENGTH][WIDTH];
StackList<Block> stack;

Block* curr_previous[64];
int curr_previous_length = 0;
Block* dest_previous[64];
int dest_previous_length = 0;
Block* path[64];
int path_length = 0;


double calculate_distance(Coordinate origin, Coordinate dest);
int run_Astar(Block* pq[]);
void visit(Block* pq[], Block b);
bool equals(Block b, Block d);
double get_distance(Block b);
bool is_inbounds(Coordinate coord);
int get_direction(Block b); // gets the direction of block b with respect to current
void find_path(Block* path[], Block dest);
void move_to_neighbor_block(Block dest);
int negate_direction(int dir);
int change_direction(int dir, int neg);

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

  // setting list to all NULLS
  reset_list(curr_previous, curr_previous_length);
  reset_list(dest_previous, dest_previous_length);
  reset_list(path, path_length);

  dest_block = {{2, 2}, false, -1, NULL};
  curr_block = {{0, 0}, false, 0, NULL};

  // create blocks to fill in grid
  for (int row = 0; row < LENGTH; row++) {
    for (int col = 0; col < WIDTH; col++) {
      if (row == dest.row && col == dest.col) {
        grid[row][col].coord.row = DEST_ROW;
        grid[row][col].coord.row = DEST_COL;
        grid[row][col].visited = false;
        grid[row][col].prev = NULL;
        grid[row][col].distance = -1;
      } else if (row == 0 && col == 0) {
        grid[row][col].coord.row = 0;
        grid[row][col].coord.col = 0;
        grid[row][col].visited = false;
        grid[row][col].prev = NULL;
        grid[row][col].distance = 0;
      } else {
        grid[row][col].coord.row = row;
        grid[row][col].coord.col = col;
        grid[row][col].visited = false;
        grid[row][col].prev = NULL;
        grid[row][col].distance = -1;
      }
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (run_once) {
    run_once = false;
    run_Astar(pq);
  }
}

int run_Astar(Block* priorityQueue[]) {
  stack.push(curr_block);
  
  // while priorityQueue not empty, go to most optimal
  while (!stack.isEmpty()) {
    Block best_block = stack.pop();

    if (best_block.visited) {
      continue;
    }

    // move robot to best_block;
    move_robot(curr_block, best_block);
    curr_block = best_block;

    // wait for robot to move to best_block?
    delay(3000);

    if (equals(best_block, dest_block)) {
      return 1;
    } else {
      visit(best_block);
    }
  }

  return 0;
}

void visit(Block b) {
  b.visited = true;
  byte curr_row = b.coord.row;
  byte curr_col = b.coord.col;
  distance = b.distance + 1;

  // for every neighbor, check if we have a shorter distance
  for (int row_offset = -1; row_offset <= 1; row_offset++) {
    for (int col_offset = -1; col_offset <= 1; col_offset++) {

      Coordinate new_coord = {curr_row + row_offset, curr_col + col_offset};
      // only check four directions, top, bot, left, right, make sure in bounds
      if (row_offset ^ col_offset && is_inbounds(new_coord)) {

        // get the neighbor from the grid, make sure it's reachable and hasn't been seen
        Block* neighbor = grid[curr_row + row_offset][curr_col + col_offset];
        if (is_reachable(neighbor) && !neighbor->visited) {
          neighbor->distance = distance;
          neighbor->prev = &b;
          // add to queue
          stack.push(*neighbor);
        }
      }
    }
  }
}


void reset_list(Block* list[], int* length) {
  for (int i = 0; i < 64; i++) list[i] = NULL;
  *length = 0;
}


void list_add(Block* list[], Block* b, int* length) {
  list[*length] = b;
  (*length)++;
}

int find_in_list(Block* list[], Block* b, int* length) {
  int curr_idx = 0;
  while (curr_idx < *list_length) {
    if (list[curr_idx] == b) {
      return curr_idx;
    }
    curr_idx++;
  }
  return -1;
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
  Block* path[64];
  for (int i = 0; i < 64; i++) path[i] = NULL;

  reset_list();
  Block* cursor = &curr;
  // fill previous list
  while (cursor != NULL) {
    list_add(cursor);
    cursor = cursor->prev;
  }

  // find a path from curr_block to dest
  find_path(path, dest);

  int next_block_idx = 0;
  while (!equals(curr_block, dest)) {
    move_to_neighbor_block(*path[next_block_idx]);
    curr_block = *path[next_block_idx];
  }
}

void find_path(Block* path[], Block dest) {
  // keep going back until we find a match in previous path
  int found_idx = -1;
  int path_length = 0;
  Block* curr_dest = &dest;

  Block* dest_previous[64];
  int dest_previous_length = 0;

  while (curr_dest != NULL) {
    dest_previous[dest_previous_length] = curr_dest;
    dest_previous_length++;
    found_idx = find_in_list(curr_dest);
    if (found_idx == -1) {
      curr_dest = curr_dest->prev;
    } else {
      break;
    }
  }

  // found_idx contains the LCA, must add to path
  for (int i = 0; i <= found_idx; i++) {
    path[path_length] = list[i];
    path_length++;
  }

  // now add the ancestors 
  for (int i = dest_previous_length - 1; i >= 0; i--) {
    path[path_length] = dest_previous[i];
    path_length++;
  }
}

void move_to_neighbor_block(Block dest) {
  int dir_to_dest = get_direction(dest);
  switch (dir_to_dest) {
    case TOP:
      if (direction == TOP) {
        move_forward_block();
      } else if (direction == BOTTOM) {
        turn_around();
        move_forward_block();
      } else if (direction == LEFT) {
        turn_right();
        move_forward_block();
      } else if (direction == RIGHT) {
        turn_left();
        move_forward_block();
      }
    case BOTTOM:
      if (direction == TOP) {
        turn_around();
        move_forward_block();
      } else if (direction == BOTTOM) {
        move_forward_block();
      } else if (direction == LEFT) {
        turn_left();
        move_forward_block();
      } else if (direction == RIGHT) {
        turn_right();
        move_forward_block();
      }
    case LEFT:
      if (direction == TOP) {
        turn_left();
        move_forward_block();
      } else if (direction == BOTTOM) {
        turn_right();
        move_forward_block();
      } else if (direction == LEFT) {
        move_forward_block();
      } else if (direction == RIGHT) {
        turn_around();
        move_forward_block();
      }
    case RIGHT:
      if (direction == TOP) {
        turn_right();
        move_forward_block();
      } else if (direction == BOTTOM) {
        turn_left();
        move_forward_block();
      } else if (direction == LEFT) {
        turn_around();
        move_forward_block();
      } else if (direction == RIGHT) {
        move_forward_block();
      }
  }
}

bool is_inbounds(Coordinate coord) {
  return (coord.row >= 0 && coord.row < LENGTH && coord.col >= 0 && coord.col < WIDTH);
}

byte get_index(Block b) {
  byte row = b.coord.row;
  byte col = b.coord.col;
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
  return sqrt(pow((origin_y - dest_y), 2) + pow((origin_x - dest_x), 2));
}

bool is_empty(Block* priorityQueue[]) {
  return size == 0;
}

Block* peek_highest_priority(Block* priorityQueue[]) {
  if (size == 0) {
    return NULL;
  } else {
    return priorityQueue[0];
  }
}

Block* pop_highest_priority(Block* priorityQueue[]) {
  if (size == 0) {
    return NULL;
  } else {
    Block* to_return = priorityQueue[0];
    priorityQueue[0] = priorityQueue[size - 1];
    sink(priorityQueue, 0);
    size--;
    return to_return;
  }
}

void pq_add(Block* priorityQueue[], Block b) {
  Serial.println("adding block to pq");
  delay(1000);
  if (size >= MAX_NUM_BLOCKS) return;
  priorityQueue[size] = &b;
  Serial.println("printing pq in pq_add");
  delay(1000);
  Serial.println(pq[0]->coord.row);
  delay(1000);
  Serial.println(pq[0]->coord.col);
  swim(priorityQueue, size);
  delay(1000);
  size++;
}

void swap(Block* priorityQueue[], byte index_one, byte index_two) {
  Block* temp = priorityQueue[index_one];
  priorityQueue[index_one] = priorityQueue[index_two];
  priorityQueue[index_two] = temp;
}

void sink(Block* priorityQueue[], byte index) {
  byte curr_index = index;

  // while we still have children in priorityQueue
  while (LEFT_CHILD_INDEX(curr_index) < size) {
    Block* curr_block = priorityQueue[curr_index];
    byte smaller_index = LEFT_CHILD_INDEX(curr_index);
    byte right_idx = (RIGHT_CHILD_INDEX(curr_index) < size) ? RIGHT_CHILD_INDEX(curr_index) : -1;
    Block* smaller_block = priorityQueue[smaller_index];
    Block* right_block = NULL;
    if (right_idx < size) {
      right_block = priorityQueue[right_idx];
    }

    if (right_block != NULL && get_distance(*right_block) < get_distance(*smaller_block)) {
      smaller_block = right_block;
    }

    if (get_distance(*smaller_block) < get_distance(*curr_block)) {
      swap(priorityQueue, smaller_index, curr_index);
      curr_index = smaller_index;
    } else {
      return;
    }
  }

  return;
}

void swim(Block* priorityQueue[], byte index) {
  Serial.println("swimming at index: ");
  Serial.println(index);
  delay(1000);

  byte curr_index = index;

  while (curr_index > 0) {
    Block* curr_block = priorityQueue[curr_index];
    byte parent_idx = PARENT_INDEX(curr_index);
    Block* parent_block = NULL;
    if (parent_idx >= 0) {
      parent_block = priorityQueue[parent_idx];
    }

    assert(parent_block);
    
    if (get_distance(*parent_block) > get_distance(*curr_block)) {
      swap(priorityQueue, parent_idx, curr_index);
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
  delay(3000);
  direction = change_direction(direction, -1);
}

void turn_right() {
  for (int i = 0; i < 180; i++) {
    rotateDeg(1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
  direction = change_direction(direction, 1);
}


void move_forward_block() {
  for (int i = 0; i < 360; i++) {
    rotateDeg(-1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
}

void turn_around() {
  for (int i = 0; i < 360; i++) {
    rotateDeg(1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
  direction = negate_direction(direction);
}

int negate_direction(int direction) {
  switch (direction) {
    case TOP:
      return BOTTOM;
    case BOTTOM:
      return TOP;
    case LEFT:
      return RIGHT;
    case RIGHT:
      return LEFT;
  }
}

int change_direction(int dir, int num) {
  // if turning right
  if (num > 0) {
    switch (dir) {
      case TOP:
        return RIGHT;
      case BOTTOM:
        return LEFT;
      case LEFT:
        return TOP;
      case RIGHT:
        return BOTTOM;
    }
  } else {
    switch (dir) {
      case TOP:
        return LEFT;
      case BOTTOM:
        return RIGHT;
      case LEFT:
        return BOTTOM;
      case RIGHT:
        return TOP;
    }
  }
}


