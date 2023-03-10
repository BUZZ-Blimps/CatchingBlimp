#include "TestLib.h"
#include <iostream>

using namespace std;

void testFunc(bool yes, float* floatPtr){
	if(yes){
		*floatPtr = 1;
	}else{
		floatPtr = nullptr;
	}
}

int main() {
	for(int i=0; i<20; i++){
		int j = Increment(i);
		cout << "Starting Num: " << i << ";  Incremented: " << j << ";" << endl;
	}

	float myFloat = 1000.0f;
	testFunc(true, &myFloat);
	cout << "MyFloat: " << myFloat << endl;
	testFunc(false, &myFloat);
	cout << "MyFloat==nullptr: " << (&myFloat == nullptr) << endl;
	if(&myFloat != nullptr){
		cout << "MyFloat: " << myFloat << endl;
	}

	return 0;
}
