#include <stdio.h>
#include <time.h>
#include <iostream>

int main(){
time_t timer;
int t;

t = time(NULL);

std::cout << t << std::endl ;
return 0;
}
