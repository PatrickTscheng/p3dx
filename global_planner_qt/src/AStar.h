/*
 * AStar.h
 *
 *  Created on: Apr 24, 2018
 *      Author: yashuai
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include<utility>
#include<limits>
#include<stdio.h>
#include <bits/stdc++.h>
#include<string.h>
#include<stdint.h>
#include<vector>



namespace global_planner {

typedef std::pair<int, int> Pair;

typedef std::pair<double, std::pair<int, int> > pPair;

// A structure to hold the neccessary parameters
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

class AStar {
public:
	AStar();
	virtual ~AStar();

private:
	// A Utility Function to check whether given cell (row, col)
	// is a valid cell or not.
	bool isValid(int row, int col, int ROW, int COL);

	// A Utility Function to check whether the given cell is
	// blocked or not
  bool isUnBlocked(const std::vector< std::vector<int> > &grid, const int row, const int col);

	// A Utility Function to check whether destination cell has
	// been reached or not
	bool isDestination(int row, int col, const Pair dest);

	// A Utility Function to calculate the 'h' heuristics.
	double calculateHValue(int row, int col, const Pair &dest);

	// A Utility Function to trace the path from the source
	// to destination
	void tracePath(cell** cellDetails, const Pair &dest, std::vector<Pair> &plan);

public:
	// A Function to find the shortest path between
	// a given source cell to a destination cell according
	// to A* Search Algorithm
  void aStarSearch(const std::vector<std::vector<int> > &grid, const Pair &start, const Pair &dest, std::vector<Pair> &plan);
};

} /* namespace global_planner */

#endif /* ASTAR_H_ */
