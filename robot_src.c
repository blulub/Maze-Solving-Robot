#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_NUM_BLOCKS 256
#define LEFT_CHILD_INDEX(x) ((2 * x))
#define RIGHT_CHILD_INDEX(x) ((2 * x + 1))
#define PARENT_INDEX(x) ((x / 2))
#define SQUARE(x) ((x * x));

typedef struct {
	int row;
	int col;
} Coordinate;

typedef struct {
	Coordinate coord;
	int visited;
	Block* prev;
	double dist;
} Block;

typedef struct {
	Block data[MAX_NUM_BLOCKS];
	Block* tail;
	int size;
} Priority_Queue;

Coordinate dest;
Coordinate origin;
Priority_Queue pq;
Coordinate current;

// map of block ids to distances
double distances[MAX_NUM_BLOCKS];


Block peek_highest_priority(Priority_Queue pq);
Block pop_highest_priority(Priority_Queue pq);
void pq_add(Priority_Queue pq, Block b);
int is_empty(Priority_Queue pq);
void sink(Priority_Queue pq, int ind);
void swim(Priority_Queue pq, int ind);
void swap(Block blocks[], int index_one, int index_two);
double get_distance(Coordinate origin, Coordinate dest);
void run_Astar(Priority_Queue pq);
void visit(Priority_Queue pq, Block b);
int equals(Block b, Block d);
int get_index(Block b);

// TODO: Implement all functions below

/* Returns if the dest block is reachable given the current block */
int is_reachable(Block dest);
void move(Block dest); /* Moves robot to dest block from current block */

// END TODO

void setup() {
	memset(&distances, -1, MAX_NUM_BLOCKS);

	dest = {.row = 0, .col = 0}; // reset this before solving maze
	origin = {.row = 0, .col = 0}; // origin is always 0, 0
	current = {.row = 0, .col = 0};
	pq = {}; // initialize empty priority queue
}

void loop() {
  // put your main code here, to run repeatedly:

}

void run_Astar(Priority_Queue pq) {
	Block start_block = {origin, 0, NULL, get_distance(origin, dest)};
	pq.pq_add(start_block);

	// while pq not empty, go to most optimal
	while (!is_empty(pq)) {

	}
}

int get_index(Block b) {
	int row = b.coord.row;
	int col = b.coord.col;

	
}

int equals(Block b, Block d) {
	return b.coord.row == d.coord.row && b.coord.col == d.coord.col;
}

double square(double num) {
	return num * num;
}

double get_distance(Coordinate origin, Coordinate dest) {
	double origin_y = (double) origin.row;
	double origin_x = (double) origin.col;
	double dest_y = (double) dest.row;
	double dest_x = (double) dest.col;
	return sqrt(SQUARE(origin_y - dest_y) + SQUARE(origin_x - dest_x));
}

int is_empty(Priority_Queue pq) {
	return !pq.size;
}

Block peek_highest_priority(Priority_Queue pq) {
	if (pq.size == 0) {
		return NULL;
	} else {
		return pq.data[0];
	}
}

Block pop_highest_priority(Priority_Queue pq) {
	if (pq.size == 0) {
		return NULL;
	} else {
		Block to_return = pq.data[0];
		pq.data[0] = pq.data[pq.size - 1];
		sink(pq, 0);
		return to_return;
	}
}

void pq_add(Priority_Queue pq, Block b) {
	if (pq.size >= MAX_NUM_BLOCKS) return;
	pq.data[pq.size] = b;
	swim(pq, pq.size);
	pq.size++;
}

void swap(Priority_Queue pq, int index_one, int index_two) {
	Block temp = pq.data[index_one];
	pq.data[index_one] = pq.data[index_two];
	pq.data[index_two] = temp;
}

void sink(Priority_Queue pq, int index) {
	int curr_index = index;

	// while we still have children in pq
	while (LEFT_CHILD_INDEX(curr_index) < pq.size) {
		int smaller_index = LEFT_CHILD_INDEX(curr_index);
		int right_idx = (RIGHT_CHILD_INDEX(curr_index) < pq.size) ? RIGHT_CHILD_INDEX(curr_index) : -1;
		if (right_idx > -1 && (pq.data[right_idx].dist < pq.data[smaller_index].dist) {
			smaller_index = right_idx;
		}

		if (pq.data[smaller_index].dist < pq.data[curr_index].size) {
			swap(pq, smaller_index, curr_index);
			curr_index = smaller_index;
		} else {
			return;
		}
	}

	return;
}

void swim(Priority_Queue pq, int index) {
	int curr_index = index;

	while (curr_index > 0) {
		int parent_idx = PARENT_INDEX(curr_index);
		if (pq.data[parent_idx].dist < pq.data[curr_index].dist) {
			swap(pq, parent_idx, curr_index);
			curr_index = parent_idx;
		} else {
			return;
		}
	}

	return;
}














