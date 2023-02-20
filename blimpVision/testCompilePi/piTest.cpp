#include "TestLib.h"
#include <iostream>

using namespace std;

int main() {
	for(int i=0; i<20; i++){
		int j = Increment(i);
		cout << "Starting Num: " << i << ";  Incremented: " << j << ";" << endl;
	}
	return 0;
}
