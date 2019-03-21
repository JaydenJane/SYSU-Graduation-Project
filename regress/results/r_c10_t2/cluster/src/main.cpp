#include <iostream>
#include "kmeans.h"
#include "dataRead.h"

using namespace std;

int main() {
	dataInit("data.txt");
	centerInit();
	kmeans();
	resultOut();
	return 0;
}