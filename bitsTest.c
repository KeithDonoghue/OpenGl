#define count(x,y)  count_bitsx(y)
#define SIZE 2000056

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>



struct node {
  struct node* next;
  uint8_t value;
};

struct stack {
  struct node* base;
  struct node* top;

};





int count_bits1(uint8_t num){
  int i;
  int count = 0;
for(i = 0 ; i < 8;i++){
  if (num & (1 << i))
    count ++;
 }
 return count;
}


int count_bits2(uint8_t num){
  int i;
  int count = 0;
  while(num){
  if (num & 1 )
    count ++;
  num = num >> 1;
 }
 return count;
}

int count_bits3(uint8_t num){
  int i;
  int count = 0;
  while(num){
      count ++;
    num = num & (num -1) ;

 }
 return count;
}



int count_bits3a(int num){
  int i;
  int count = 0;
  while(num){
      count ++;
    num = num & (num -1) ;

 }
 return count;
}



int count_bits4(uint8_t num, uint8_t* LUT){
  int count = LUT[num];
 return count;
}



int count_bits5(uint8_t num, uint8_t* LUT){
  return  *(LUT+ num);
}

uint8_t* make_lookup(){
  uint8_t* point;
  int i;
  point = (uint8_t*) malloc(256);
  for (i = 0; i < 256;i++){
    point[i] =count_bits1(i);
  }
  return point;
}


uint8_t* make32_lookup(){
  uint8_t* point;
  unsigned int i;
  point = (uint8_t*) malloc(pow(2,32));
  for (i = 0; i < pow(2,32);i++){
    point[i] =count_bits3a(i);
  }
  return point;
}

 int main(){
   int i,y;
   clock_t start, end;
   double time_spent;
   uint8_t num,count;
   uint8_t *new;
   uint8_t *new_int;
   new = make_lookup();
   printf("count_bits%d",1);
   new_int = make32_lookup();
   uint8_t array[SIZE];
   int int_array[SIZE];

   srand(time(NULL));

   for (i = 0; i < SIZE; i++){
     array[i] = rand()%256;
   }


   for (i = 0; i < SIZE; i++){
     int_array[i] = rand()%256;
   }


   start  = clock();

   for (i = 0; i < SIZE; i++){
         int x = count_bits1(array[i]);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",1,time_spent);
  


   start  = clock();

   for (i = 0; i < SIZE; i++){
         int x = count_bits2(array[i]);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",2,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
         int x = count_bits3(array[i]);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",3,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int x = count_bits4(array[i],new);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",4,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int x = count_bits5(array[i],new);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",5,time_spent);
  



   start  = clock();

   for (i = 0; i < SIZE; i++){
  int j;
  int count = 0;
  int num = array[i];
  for(j = 0 ; j < 8;j++){
    if (num & (1 << j))
      count ++;
  }
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",1,time_spent);
  


   start  = clock();

   for (i = 0; i < SIZE; i++){
  int count = 0;
  int num = array[i];
  while(num){
  if (num & 1 )
    count ++;
  num = num >> 1;
 }

   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",2,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int count = 0;
     int num = array[i];
     while(num){
       count ++;
       num = num & (num -1) ;

     }
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",3,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int x = new[array[i]];
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",4,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int x = *(new+array[i]);
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",5,time_spent);
  

   start  = clock();

   for (i = 0; i < SIZE; i++){
     int x = new_int[int_array[i]];
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",6,time_spent);
   


   start  = clock();

   for (i = 0; i < SIZE; i++){
     int count = 0;
     int num = int_array[i];
     while(num){
       count ++;
       num = num & (num -1) ;

     }
   }

   end = clock();
   time_spent = (double) (end - start) /CLOCKS_PER_SEC;
   printf("count_bits%d takes: %f \n",7,time_spent);


   return 0;
 }
