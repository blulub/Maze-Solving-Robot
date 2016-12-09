#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <StackList.h>
#include <TimerOne.h>


// Neil's hardware code
#define leftPin A2
#define rightPin A5
#define frontPin A0
#define DIR_PIN_LEFT 3
#define STEP_PIN_LEFT 7
#define DIR_PIN_RIGHT 5
#define STEP_PIN_RIGHT 4
#define ABS(x) (-x)

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


#define MAX_NUM_BLOCKS 64
#define LEFT 0
#define RIGHT 1
#define TOP 2
#define BOTTOM 3
#define LENGTH 8
#define WIDTH 8
#define DEST_ROW 2
#define DEST_COL 8

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

Block* dest_block_ptr;
Block* curr_block_ptr;
int direction = BOTTOM;
int curr_distance;
bool run_once = true;
Block grid[LENGTH][WIDTH];
StackList<Block*> stack;

Block* curr_previous[MAX_NUM_BLOCKS];
int curr_previous_length = 0;
Block* dest_previous[MAX_NUM_BLOCKS];
int dest_previous_length = 0;
Block* path[MAX_NUM_BLOCKS];
int path_length = 0;


int run_floodfill();
void visit(Block* b);
bool equals(Block* b, Block* d);
bool is_inbounds(Coordinate coord);
int get_direction(Block* b); // gets the direction of block b with respect to current
void find_path(Block* path[], Block* dest);
void move_to_neighbor_block(Block* dest);
int negate_direction(int dir);
int change_direction(int dir, int neg);
void reset_list(Block* list[], int* length);
void list_add(Block* list[], Block* val, int* length);
int find_in_list(Block* list[], Block* val, int* length);
bool is_reachable(Block dest);

/* Returns if the dest block is reachable given the current block */
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
void move_robot(Block* current, Block* dest); /* Moves robot to dest block from current block */

void setup() {
  // set up Neil's hardware code
  Serial.begin (9600);
  for(i = 0; i < 3; i++) pinMode(irPin[i], INPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(STEP_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(STEP_PIN_RIGHT, OUTPUT);
  Timer1.initialize(500000);
  Timer1.attachInterrupt(readIRValue);

  // end Neil's setup

  // setting list to all NULLS
  direction = BOTTOM;
  reset_list(curr_previous, &curr_previous_length);
  reset_list(dest_previous, &dest_previous_length);
  reset_list(path, &path_length);

  // create blocks to fill in grid
  for (int row = 0; row < LENGTH; row++) {
    for (int col = 0; col < WIDTH; col++) {
      if (row == DEST_ROW && col == DEST_COL) {
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

  curr_block_ptr = &(grid[0][0]);
  dest_block_ptr = &(grid[DEST_ROW][DEST_COL]);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting Loop");
  delay(1000);

  if (run_once) {
    run_once = false;
    int result = run_floodfill();
    Serial.println("finished with response of: ");
    delay(1000);
    Serial.println(result);
    delay(1000);
  }
}

int run_floodfill() {
  Serial.println("Starting floodfill");
  delay(1000);

  // push starting block onto stack
  stack.push(curr_block_ptr);

  Serial.println("Pushed starting block onto stack: ");
  delay(1000);
  Serial.println(curr_block_ptr->coord.row);
  delay(1000);
  Serial.println(curr_block_ptr->coord.col);

  // while stack not empty, go to most optimal
  while (!stack.isEmpty()) {
    Block* best_block_ptr = stack.pop();
    if (best_block_ptr->visited) continue;
    // move robot to best_block;
    move_robot(curr_block_ptr, best_block_ptr);
    delay(3000);
    curr_block_ptr = best_block_ptr;
    // wait for robot to move to best_block?
    if (equals(best_block_ptr, dest_block_ptr)) {
      return 1;
    } else {
      visit(best_block_ptr);
      delay(2000);
    }
  }
  return 0;
}

void visit(Block* b) {
  b->visited = true;
  int curr_row = b->coord.row;
  int curr_col = b->coord.col;
  curr_distance = b->distance;

  // for every neighbor, check if we have a shorter distance
  for (int row_offset = -1; row_offset <= 1; row_offset++) {
    for (int col_offset = -1; col_offset <= 1; col_offset++) {

      Coordinate new_coord = {curr_row + row_offset, curr_col + col_offset};
      // only check four directions, top, bot, left, right, make sure in bounds
      if ((row_offset == 0 || col_offset == 0) && (row_offset != 0 && col_offset != 0) && is_inbounds(new_coord)) {

        // get the neighbor from the grid, make sure it's reachable and hasn't been seen
        Block neighbor = grid[curr_row + row_offset][curr_col + col_offset];
        Serial.println("checking neighbor at: ");
        delay(1000);
        Serial.println(curr_row + row_offset);
        delay(1000);
        Serial.println(curr_col + col_offset);
        delay(1000);

        if (is_reachable(neighbor) && !neighbor.visited) {
          neighbor.distance = curr_distance + 1;
          neighbor.prev = b;
          // add to stack
          stack.push(&neighbor);
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
  while (curr_idx < *length) {
    if (list[curr_idx] == b) {
      return curr_idx;
    }
    curr_idx++;
  }
  return -1;
}

// determines if a neighboring block is reachable
bool is_reachable(Block b) {
  Serial.println("current block at: ");
  delay(1000);
  Serial.println(curr_block_ptr->coord.row);
  delay(1000);
  Serial.println(curr_block_ptr->coord.col);
  Serial.println("checking if neighbor is open");
  delay(1000);
  Serial.println(b.coord.row);
  delay(1000);
  Serial.println(b.coord.col);
  delay(1000);


  int dir_block_b = get_direction(&b);

  Serial.println("direction of neighbor to current: ");
  delay(1000);
  Serial.println(dir_block_b);
  delay(1000);

  Serial.println("current direction facing: ");
  delay(1000);
  Serial.println(direction);
  delay(1000);
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

void move_robot(Block* curr_ptr, Block* dest_ptr) {

  Serial.println("moving from current block:");
  delay(1000);
  Serial.println(curr_ptr->coord.row);
  delay(1000);
  Serial.println(curr_ptr->coord.col);
  delay(1000);
  Serial.println("moving to dest block:");
  delay(1000);
  Serial.println(dest_ptr->coord.row);
  delay(1000);
  Serial.println(dest_ptr->coord.col);
  delay(1000);


  if (curr_ptr == dest_ptr) return;
  // use previous fields to find a path from curr to dest
  reset_list(curr_previous, &curr_previous_length);
  Block* cursor = curr_block_ptr;
  // fill previous list
  while (cursor != NULL) {
    list_add(curr_previous, cursor, &curr_previous_length);
    cursor = cursor->prev;
  }

  // find a path from curr_block to dest
  find_path(path, dest_ptr);

  int next_block_idx = 0;
  while (!equals(curr_block_ptr, dest_ptr)) {
    move_to_neighbor_block(path[next_block_idx]);
    curr_block_ptr = path[next_block_idx];
    next_block_idx++;
  }
}

void find_path(Block* path[], Block* dest) {
  // keep going back until we find a match in previous path
  reset_list(dest_previous, &dest_previous_length);
  reset_list(path, &path_length);
  int found_idx = -1;

  Block* curr_dest = dest;
  while (curr_dest != NULL) {
    list_add(dest_previous, curr_dest, &dest_previous_length);
    found_idx = find_in_list(curr_previous, curr_dest, &curr_previous_length);
    if (found_idx == -1) {
      curr_dest = curr_dest->prev;
    } else {
      break;
    }
  }

  // found_idx contains the LCA, must add to path
  for (int i = 0; i <= found_idx; i++) {
    list_add(path, curr_previous[i], &path_length);
  }

  // now add the ancestors of dest
  for (int i = dest_previous_length - 1; i >= 0; i--) {
    list_add(path, dest_previous[i], &path_length);
  }
}

void move_to_neighbor_block(Block* dest) {
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

bool equals(Block* b, Block* d) {
  return b->coord.row == d->coord.row && b->coord.col == d->coord.col;
}

int get_direction(Block* b) {
  int curr_row = curr_block_ptr->coord.row;
  int curr_col = curr_block_ptr->coord.col;
  int b_row = b->coord.row;
  int b_col = b->coord.col;

  if (b_col == curr_col) {
    if (b_row > curr_row) {
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
  Serial.println("turning left");
  delay(1000);

  for (int i = 0; i < 180; i++) {
    rotateDeg(-1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(-1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
  direction = change_direction(direction, -1);
}

void turn_right() {
  Serial.println("turning right");
  delay(1000);

  for (int i = 0; i < 180; i++) {
    rotateDeg(1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
  direction = change_direction(direction, 1);
}


void move_forward_block() {
  Serial.println("moving forward block");
  delay(1000);

  for (int i = 0; i < 360; i++) {
    rotateDeg(-1, 0.1, DIR_PIN_LEFT, STEP_PIN_LEFT);
    rotateDeg(1, 0.1, DIR_PIN_RIGHT, STEP_PIN_RIGHT);
  }
  delay(3000);
}

void turn_around() {
  Serial.println("turning around");
  delay(1000);

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


