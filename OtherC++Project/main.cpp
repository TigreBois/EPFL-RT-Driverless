#include<iostream>
using namespace std;
int GreatestOfThree(int a,int b,int c){
  if((a>b) && (a>c)){    //for a > b and a>c case
    return a;
   }
   else if(b>c){    //for b>c case 
    return b;
   }
  else{
    return c;
  }
 return 0;
}
//After checking for correctness comment main function to test //main_test.cpp
/*int main()
{
 // your code goes here
 int a,b,c;
 cin>>a>>b>>c;
 cout<<GreatestOfThree(a,b,c);
 return 0;
}*/