#pragma once
#include <string>



const int WINDOW_SIZE = 10;
extern double MIN_PARALLAX;


//feature tracker section
extern int ROW;
extern int COL;
extern int EQUALIZE;


extern bool PUB_THIS_FRAME;
extern int FISHEYE;
extern std::string FISHEYE_MASK;
extern double FOCAL_LENGTH;
extern double F_THRESHOLD;
extern int MAX_CNT;
extern int MIN_DIST;