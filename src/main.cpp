// please use "-std=c++11" for initialization.


#include <iostream>
#include "Hungarian.h"


int main(void)
{

	/*
		This example: NxM, where N = M
		N rows assigned to M cols
	*/
    
	vector< vector<double> > costMatrix = { 
										  { 10, 19, 8, 15 }, 
										  { 1, 18, 7, 17 }, 
										  { 13, 16, 9, 14 }, 
										  { 12, 3, 8, 18 } };

	HungarianAlgorithm HungAlgo;
	vector<int> assignment;

	double cost = HungAlgo.Solve(costMatrix, assignment);

	for (unsigned int x = 0; x < costMatrix.size(); x++)
		std::cout << x << "," << assignment[x] << "\t";

	std::cout << "\ncost: " << cost << std::endl;

	return 0;
}
