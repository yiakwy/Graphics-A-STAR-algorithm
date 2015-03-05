#include "GameMap.h"

#include <windows.h>
#include <stdio.h>

#include <iostream>

void main(){
	freopen("test_cases.txt", "r", stdin);

	Orienteering o;

	o.main();
	
	printf("end of the program");
}