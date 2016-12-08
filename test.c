#include <stdio.h>
#include <stdbool.h>

// defines a coordinate
typedef struct Coordinate {
	int row;
	int col;
} Coordinate;

// defines a block
typedef struct Block {
  struct Coordinate coord;
  bool visited;
  struct Block* prev;
  int distance;
} Block;

// makes a global grid of blocks
Block grid[8][8];
Block* stack[8];
int length;



// fills in the global grid of blocks
void fillGrid();
void changeGrid();


void fillGrid(int* length) {
	// makes a global grid of blocks

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			grid[i][j].coord.row = i;
			grid[i][j].coord.col = j;
			(*length)++;
		}
	}
}

void changeGrid() {
	grid[0][0].coord.row = -1;
	grid[0][0].coord.col = -1;
	length = 10;
}

int main() {
	// fill the global grid, then print it
	fillGrid(&length);
	stack[0] = &(grid[0][0]);
	stack[0]->coord.row = 5;
	changeGrid();
	printf("%p, %p\n", stack[0], &(grid[0][0]));
	printf("%d, %d\n", (grid[2][2]).coord.row, (grid[2][2]).coord.col);
	printf("%d, %d\n", (grid[0][0]).coord.row, (grid[0][0]).coord.col);
	printf("length: %d\n", length);
}




