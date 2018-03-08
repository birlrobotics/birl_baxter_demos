#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include<assert.h>
using namespace std;
#ifndef COLOR_H
#define	COLOR_H
class ColorRGB{
  public:
   float r,g,b;
   float H,S,V;
   ColorRGB()
   {
     
   }
   ColorRGB(float rgb)
   {
       int rgbi=*reinterpret_cast<int*>(&rgb);
       parseColorRGB(rgbi);
       convertToHSV();
   }

   ColorRGB(float r_, float g_, float b_)
   {
       r=r_;
       g=g_;
       b=b_;
       assert(r<=1&&r>=0);
       assert(g<=1&&g>=0);
       assert(b<=1&&b>=0);
       convertToHSV();
   }

   ColorRGB(int rgbi)
   {
       parseColorRGB(rgbi);
       convertToHSV();
   }


   static float distance(ColorRGB c1,ColorRGB c2)
   {
       return sqrt(pow(c1.r-c2.r,2)+pow(c1.g-c2.g,2)+pow(c1.b-c2.b,2));
   }
   static float HSVdistance(ColorRGB c1,ColorRGB c2)
   {
       return min(abs(c1.H - c2.H), min(abs(c1.H + 360 - c2.H), abs(c2.H + 360 - c1.H))) ;
   }

   void parseColorRGB(int rgbi)
   {
       int ri=(rgbi&(0xff0000))>>16;
       int gi=(rgbi&(0xff00))>>8;
       int bi=(rgbi&(0xff));
       r=ri/255.0;
       g=gi/255.0;
       b=bi/255.0;
   }
   void assignColor(float r_, float g_, float b_)
   {
       r=r_;
       g=g_;
       b=b_;
       assert(r<=1&&r>=0);
       assert(g<=1&&g>=0);
       assert(b<=1&&b>=0);
       convertToHSV();
   }
   
   void assignColor(float rgb){
       int rgbi=*reinterpret_cast<int*>(&rgb);
       parseColorRGB(rgbi);
       convertToHSV();
   }
   
   void convertToHSV()
   {
    double maxC = b;
	if (maxC < g) maxC = g;
	if (maxC < r) maxC = r;
	double minC = b;
	if (minC > g) minC = g;
	if (minC > r) minC = r;

	double delta = maxC - minC;

	V = maxC;
	S = 0;
	H = 0;

	if (delta == 0)
	{
		H = 0;
		S = 0;
	}
	else
	{
		S = delta / maxC;
		double dR = 60*(maxC - r)/delta + 180;
		double dG = 60*(maxC - g)/delta + 180;
		double dB = 60*(maxC - b)/delta + 180;
		if (r == maxC)
			H = dB - dG;
		else if (g == maxC)
			H = 120 + dR - dB;
		else
			H = 240 + dG - dR;
	}
	if (H<0)
		H+=360;
	if (H>=360)
		H-=360;

        assert(H>=0 && H<=360);


   }

   float getFloatRep()
   {
       int color=(((int)(r*255))<<16)+(((int)(g*255))<<8)+(((int)(b*255)));
       return *reinterpret_cast<float*>(&color);
   }

   float squaredError(ColorRGB c)
   {
       return pow(r-c.r,2)+pow(g-c.g,2)+pow(b-c.b,2);
   }

   void print()
   {
       std::cerr<<r*255<<" "<<g*255<<" "<<b*255<<endl;
   }
};
#endif	/* COLOR_H */
