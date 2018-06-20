/*
 * AStar.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: yashuai
 */

#include "global_planner_qt/AStar.h"
#include "global_planner_qt/param.h"

namespace global_planner {

AStar::AStar() {
	// TODO Auto-generated constructor stub
}

AStar::~AStar() {
	// TODO Auto-generated destructor stub
}

bool AStar::isValid(int row, int col, int ROW, int COL)
{
    // Returns true if ROW number and COLumn number
    // is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

bool AStar::isUnBlocked(const std::vector<std::vector<int> > &grid, const int row, const int col)
{
    // Returns true if the cell is not blocked else false
    if (grid[row][col] == 100 || grid[row][col] == 110)
        return false;
    else
        return true;
}

bool AStar::isDestination(int row, int col, const Pair dest)
{
    if (row == dest.first && col == dest.second)
        return true;
    else
        return false;
}

double AStar::calculateHValue(const std::vector<std::vector<int> > &grid, int row, int col, const Pair &dest)
{
    // Return using the distance formula
    double HValue = ((double)sqrt ((row - dest.first)*(row - dest.first)
                                   + (col - dest.second)*(col - dest.second)));
    if (grid[row][col] >= 120) {
      HValue += 20.0 + grid[row][col] - 120;
    }
    return HValue;
}

void AStar::tracePath(cell** cellDetails, const Pair &dest, std::vector<Pair> &plan)
{
    printf ("\nThe Path is ");
    plan.clear();
    int row = dest.first;
    int col = dest.second;

    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col))
    {
        plan.push_back(std::make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    plan.push_back(std::make_pair (row, col));
}

bool AStar::aStarSearch(const std::vector<std::vector<int> > &grid, const Pair &start, const Pair &dest, std::vector<Pair> &plan)
{
	int ROW = grid.size();
	int COL = grid[0].size();

    // If the destination is out of range
    if (isValid (dest.first, dest.second, ROW, COL) == false)
    {
        printf ("Destination is invalid\n");
        return false;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(grid, dest.first, dest.second) == false)
    {
        printf ("the destination is blocked\n");
        return false;
    }
    /**
    if (isUnBlocked(grid, start.first, start.second) == false)
    {
      printf ("Source is blocked\n");

      return false;
    }
    */
    // If the destination cell is the same as source cell
    if (isDestination(start.first, start.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return false;
    }

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    std::memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    cell** cellDetails;
    cellDetails = new cell*[ROW];
    for (int i = 0; i < ROW; i++) {
    	cellDetails[i] = new cell[COL];
    }

    int i, j;

    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            cellDetails[i][j].f = std::numeric_limits<float>::max();
            cellDetails[i][j].g = std::numeric_limits<float>::max();
            cellDetails[i][j].h = std::numeric_limits<float>::max();
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initializing the parameters of the starting node
    i = start.first, j = start.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;


    //std::vector<pPair> openList;
    std::priority_queue<pPair, std::vector<pPair>, CompareDist > pq;
    // Put the starting cell on the open list and set its
    // 'f' as 0
    //pq.push(std::make_pair (0.0, std::make_pair (i, j)));
    pq.push(std::make_pair (0.0, std::make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!pq.empty())
    {
        //pPair p = *openList.begin();
        pPair p = pq.top();
        pq.pop();
        // Remove this vertex from the open list
        //openList.erase(openList.begin());

        // Add this vertex to the open list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest, plan);
                foundDest = true;
                return true;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j] == false &&
                     isUnBlocked(grid, i-1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (grid, i-1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j].f == std::numeric_limits<float>::max() ||
                        cellDetails[i-1][j].f > fNew)
                {

                    pq.push(std::make_pair(fNew, std::make_pair(i-1, j)));

                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest, plan);
                foundDest = true;
                return true;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(grid, i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(grid, i+1, j, dest);
                fNew = gNew + hNew;

                if (cellDetails[i+1][j].f == std::numeric_limits<float>::max() ||
                        cellDetails[i+1][j].f > fNew)
                {
                    pq.push(std::make_pair (fNew, std::make_pair (i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid (i, j+1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j+1] == false &&
                     isUnBlocked (grid, i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (grid, i, j+1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j+1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i][j+1].f > fNew)
                {
                    pq.push(std::make_pair(fNew, std::make_pair (i, j+1)));

                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j-1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j-1] == false &&
                     isUnBlocked(grid, i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(grid, i, j-1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j-1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i][j-1].f > fNew)
                {
                    pq.push(std::make_pair (fNew, std::make_pair (i, j-1)));

                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j+1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j+1].parent_i = i;
                cellDetails[i-1][j+1].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j+1] == false &&
                     isUnBlocked(grid, i-1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(grid, i-1, j+1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i-1][j+1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i-1][j+1].f > fNew)
                {
                    pq.push(std::make_pair (fNew, std::make_pair(i-1, j+1)));

                    // Update the details of this cell
                    cellDetails[i-1][j+1].f = fNew;
                    cellDetails[i-1][j+1].g = gNew;
                    cellDetails[i-1][j+1].h = hNew;
                    cellDetails[i-1][j+1].parent_i = i;
                    cellDetails[i-1][j+1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i-1, j-1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination (i-1, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j-1].parent_i = i;
                cellDetails[i-1][j-1].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j-1] == false &&
                     isUnBlocked(grid, i-1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(grid, i-1, j-1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i-1][j-1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i-1][j-1].f > fNew)
                {
                    pq.push(std::make_pair (fNew, std::make_pair (i-1, j-1)));
                    // Update the details of this cell
                    cellDetails[i-1][j-1].f = fNew;
                    cellDetails[i-1][j-1].g = gNew;
                    cellDetails[i-1][j-1].h = hNew;
                    cellDetails[i-1][j-1].parent_i = i;
                    cellDetails[i-1][j-1].parent_j = j;
                }
            }
        }

        //----------- 7th Successor (South-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j+1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j+1].parent_i = i;
                cellDetails[i+1][j+1].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j+1] == false &&
                     isUnBlocked(grid, i+1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(grid, i+1, j+1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i+1][j+1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i+1][j+1].f > fNew)
                {
                    pq.push(std::make_pair(fNew, std::make_pair (i+1, j+1)));

                    // Update the details of this cell
                    cellDetails[i+1][j+1].f = fNew;
                    cellDetails[i+1][j+1].g = gNew;
                    cellDetails[i+1][j+1].h = hNew;
                    cellDetails[i+1][j+1].parent_i = i;
                    cellDetails[i+1][j+1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i+1, j-1, ROW, COL) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j-1].parent_i = i;
                cellDetails[i+1][j-1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest, plan);
                foundDest = true;
                return true;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j-1] == false &&
                     isUnBlocked(grid, i+1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(grid, i+1, j-1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i+1][j-1].f == std::numeric_limits<float>::max() ||
                        cellDetails[i+1][j-1].f > fNew)
                {
                    pq.push(std::make_pair(fNew, std::make_pair(i+1, j-1)));

                    // Update the details of this cell
                    cellDetails[i+1][j-1].f = fNew;
                    cellDetails[i+1][j-1].g = gNew;
                    cellDetails[i+1][j-1].h = hNew;
                    cellDetails[i+1][j-1].parent_i = i;
                    cellDetails[i+1][j-1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (foundDest == false) {
        printf("Failed to find the Destination Cell\n");
        return false;
    }
    return true;
}

} /* namespace global_planner */





