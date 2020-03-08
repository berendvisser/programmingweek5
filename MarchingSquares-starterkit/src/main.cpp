//==============================================================
// Filename :main.cpp
// Authors :Berend Visser
//==============================================================
// Program that draws contour lines of scalar potential field.
// Pressing 'm' or 's' switches between the marching ('m') and scanning ('s') algorithms.
// Pressing 'm' or 's' a second time switches to the multithreaded or improved version of the algorithms.
// Also prints the time needed to draw one frame to stdout.

/*
 *  Created on: May 14, 2018
 *      Author: Johan Engelen
 *  Ported from D source by same author.
 */

#include "UI.h"

#include <SDL2/SDL.h>

#include <chrono>
#include <iostream>
#include <vector>



//Personal used libs
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <array>
#include <thread>

#define NUM_OF_PEAKS 2
#define SKIP_NUM_ROWS 10
#define BLOCK_LENGTH 150000




constexpr bool capFramerate = false;
//constexpr float thresholds[] = [0.015];
constexpr float thresholds[] = {0.015, 0.02, 0.03};

struct Pixel {
    int x =0;
    int y =0;

    bool operator== (const Pixel& tmpPixel) const {
        return (x == tmpPixel.x) && (y == tmpPixel.y);
    }
    bool operator< (const Pixel& rhs) const {                   //Overload operator so map can deal with pixels
        return (x < rhs.x) || (x == rhs.x && y < rhs.y);
    }
};

struct Point
{
    float x;
    float y;

    bool operator< (const Point& rhs) const {
        return (x < rhs.x) || (x == rhs.x && y < rhs.y);
    }
};

struct Blob
{
    float wells[2][2] = {{-100, -50}, {100, 50}};

    float potential(Point p)
    {
        return potential(p.x, p.y);
    }

    float potential(float x, float y)
    {
        const float scale = 100;
        float potential = 0.0f;
        for (auto well : wells) {
            potential += scale / (pow(x - well[0], 2) + pow(y - well[1], 2));
        }
        return potential;
    }

    void update(float time)
    {
        // Move the wells a little
        wells[0][0] = -100 + 30 * sinf(time / 7);
        wells[0][1] = -50 * cosf(time / 5);
        wells[1][0] = -wells[0][0];
        wells[1][1] = -wells[0][1];
    }
};

//function prototypes
Pixel searchThresholdPixel(UI& ui, Blob& blob, float threshold = 0.015);
bool checkPixel(std::vector<Pixel>& tmpWorklist, Pixel tmpPixel, Blob& blob, float threshold);
void addNeighbourPixels(Pixel tmpPixel, std::vector<Pixel>& tmpWorklist);
Pixel searchThresholdPixelBetter(UI& ui, Blob& blob, float threshold, bool reverseScan);
void testThread(int tmpValue);
void fillBufferWithdata(Blob& blob, bool* pixels, float threshold, int totalThreads, int threadNumber);

/// Scans full screen area. Complexity?
void drawContourScanning(UI &ui, Blob &blob, float threshold = 0.015)
{
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;

    // YOUR CODE HERE
    std::cout << "running Scanning\n";
    Point curPoint;  //previous point

    curPoint.x = (float)-sizeX; //first point to check against x coordinate
    curPoint.y = (float)-sizeY; //first point to check against y coordinate

    bool curPotOverThres;   //current potential of point                     
    bool prevPotOverThres;  //potential of previous point
    
    prevPotOverThres = blob.potential(curPoint) > threshold;    //set potential of first point            
    

    

    for (int y = -sizeY/2+1; y < sizeY/2; y++)  //loop through columns (y axis)
    {
        for (int x = -sizeX/2; x < sizeX/2; x++)    //loop through colums (x axis)
        {
            curPoint.x = (float)x;  //set x of current point
            curPoint.y = (float)y;   //set y of current point
                      
            curPotOverThres = blob.potential(curPoint)>threshold;   //find current potential
            
            
            if (curPotOverThres != prevPotOverThres)    //check if one of both points is over threshold    
            {
                ui.drawPixel((int)curPoint.x, (int)curPoint.y); //draw blob
            }

            prevPotOverThres = curPotOverThres; //set net prev potential                
        }
    } 
}



/// Scans full screen area multithreaded. Complexity?
void drawContourScanningThreaded(UI &ui, Blob &blob, float threshold = 0.015)
{   
    
    // YOUR CODE HERE
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;
    const int offset = (sizeX * sizeY) / 2;

    
    bool* bufferPixels; //Visted list (pointer to map of pixels on screen)
    bufferPixels = new bool[sizeX * sizeY]; //allocate memory 

    //set all values of map to zero
    for (int i = 0; i < sizeX * sizeY; i++) {
        bufferPixels[i] = false;
    }

    //std::thread ab(testThread, 10);
    //fillBufferWithdata(ui, blob, bufferPixels, 0, threshold);
    std::thread a(fillBufferWithdata,  blob, bufferPixels, threshold, 4, 0);
    std::thread b(fillBufferWithdata, blob, bufferPixels, threshold,4,1);
    std::thread c(fillBufferWithdata, blob, bufferPixels,  threshold,4,2);
    std::thread d(fillBufferWithdata, blob, bufferPixels,  threshold,4,3);

    a.join();
    b.join();
    c.join();
    d.join();

    for (int y = -sizeY / 2+1 ; y < sizeY / 2; y++)  //loop through columns (y axis)
    {
        for (int x = -sizeX / 2; x < sizeX / 2; x++)    //loop through colums (x axis)
        {
           if (bufferPixels[x + sizeX * y + offset])    //check if current point is bigger than previous and bigger than threshold    
            {
               ui.drawPixel(x, y);
            }

                          
        }
    }

    

    delete[] bufferPixels;    //free allocated memory
    
            
}

/// Scans screen area until it finds a pixel on the edge.
/// From then on, uses marching squares algorithm to find other pixels on the edge.
/// Caveat: only draws one of the potentially more than one contour losed loops.
/// Complexity?
void drawContourMarching(UI &ui, Blob &blob, float threshold = 0.015)
{
    // YOUR CODE HERE
    const int sizeX = ui.sizeX; //size of screen in x direction
    const int sizeY = ui.sizeY; //size of screen in y direction
    const int offset = (sizeX * sizeY) / 2; //offset for making center pixel 0,0

    std::cout << "running Marching\n"; //log

    std::vector<Pixel> workList;    //Make worklist a vector

    bool* visitedPixels; //Visted list (pointer to map of pixels on screen)
    visitedPixels = new bool[sizeX*sizeY]; //allocate memory 

    //set all values of map to zero
    for (int i = 0; i < sizeX * sizeY; i++) {
        visitedPixels[i] = false;
    }    
    
    workList.push_back(searchThresholdPixel(ui, blob, threshold));  //Find pixel on curve and put on worklist
    

    while (!workList.empty()) //Check if worklist is empty
    {          
        Pixel currentPixel = workList.back();  //find last pixel worklist and remove it
        workList.pop_back();    //remove pixel from worklist
        

        if (visitedPixels[currentPixel.x + sizeX*currentPixel.y+offset]) {     //Check if pixel is already visited using offset            
            continue; //skip to next iteration of loop
        }
        else {            
            visitedPixels[currentPixel.x + sizeX * currentPixel.y + offset] = true; //Add pixel to visited list;
        }
        if (checkPixel(workList, currentPixel, blob, threshold)) { //check if pixel is on curve
            addNeighbourPixels(currentPixel, workList); //add neighbours of that pixel to worklist
            ui.drawPixel(currentPixel.x, currentPixel.y);   //drawpixel
        }
    } 
    delete [] visitedPixels;    //free allocated memory
}

/// Improved marching squares algorithm.
/// Should have better complexity than the initial algorithm described in the manual :)
void drawContourMarchingBetter(UI& ui, Blob& blob, float threshold = 0.015)
{
    // YOUR CODE HERE
        // YOUR CODE HERE
    const int sizeX = ui.sizeX; //size of screen in x direction
    const int sizeY = ui.sizeY; //size of screen in y direction
    const int offset = (sizeX * sizeY) / 2; //offset for making center pixel 0,0

    std::cout << "running Marching++\n"; //log

    std::vector<Pixel> workList;    //Make worklist a vector

    bool* visitedPixels; //Visted list (pointer to map of pixels on screen)
    visitedPixels = new bool[sizeX * sizeY]; //allocate memory 

    //set all values of map to zero
    for (int i = 0; i < sizeX * sizeY; i++) {
        visitedPixels[i] = false;
    }

    for (int i = 0; i < NUM_OF_PEAKS ; i++)
    {
        workList.push_back(searchThresholdPixelBetter(ui, blob, threshold, i));  //Find pixel on curve and put on worklist


        while (!workList.empty()) //Check if worklist is empty
        {
            Pixel currentPixel = workList.back();  //find last pixel worklist and remove it
            workList.pop_back();    //remove pixel from worklist


            if (visitedPixels[currentPixel.x + sizeX * currentPixel.y + offset]) {     //Check if pixel is already visited using offset            
                continue; //skip to next iteration of loop
            }
            else {
                visitedPixels[currentPixel.x + sizeX * currentPixel.y + offset] = true; //Add pixel to visited list;
            }
            if (checkPixel(workList, currentPixel, blob, threshold)) { //check if pixel is on curve
                addNeighbourPixels(currentPixel, workList); //add neighbours of that pixel to worklist
                ui.drawPixel(currentPixel.x, currentPixel.y);   //drawpixel
            }
        }
    }
    delete[] visitedPixels;    //free allocated memory
}



int main(int /*argc*/, char ** /*argv*/)
{
    using namespace std::chrono;

    UI gui(1000, 600);
    Blob blob;

    unsigned long frameCounter = 0;
    auto drawContour = &drawContourScanning;

    bool pause = false;
    bool quit = false;
    while (!quit)
    {
        // Set timeout to limit frame rate
        auto timeout = SDL_GetTicks() + 20;

        if (!pause)
        {
            blob.update(frameCounter++ / 2.0);
        }

        gui.clear();


        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        for (auto t: thresholds)
        {
            drawContour(gui, blob, t);
        }
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        gui.present();
        auto duration = duration_cast<microseconds>(t2 - t1).count();
        std::cout << duration << " ms" << std::endl;

        // Handle the input.
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            // Quit button.
            if (event.type == SDL_QUIT)
            {
                quit = true;
            }
            // All keydown events
            if (event.type == SDL_KEYDOWN)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                    quit = true;
                    break;
                case SDLK_m:
                    drawContour = (drawContour == &drawContourMarching) ? &drawContourMarchingBetter
                        : &drawContourMarching;
                    break;
                case SDLK_s:
                    drawContour = (drawContour == &drawContourScanning) ? &drawContourScanningThreaded
                        : &drawContourScanning;
                    break;
                case SDLK_SPACE:
                    pause = !pause;
                    break;
                default:
                    break;
                }
            }
        }
        while (capFramerate && !SDL_TICKS_PASSED(SDL_GetTicks(), timeout))
        {
            // do nothing
        }
    }

    return 0;
}

//This functions returns the first pixel on the curve of the vector field it finds, if nothing is found it return the standard 0,0 pixel
Pixel searchThresholdPixel(UI& ui, Blob& blob, float threshold)
{ 
    const int sizeX = ui.sizeX; //find screen size
    const int sizeY = ui.sizeY; //" "
    Pixel tmpPixel;

    for (int y = -sizeY / 2; y < sizeY / 2; y ++) //loop through rows (y coordinates)
    {
        for (int x = -sizeX/2; x < sizeX/2; x++) //loop through rows (x coordinates)
        {
            if (blob.potential((float)x, (float)y) > threshold) //if potential is more than threshold
            {
                tmpPixel.x = x; //set pixel to 
                tmpPixel.y = y; //set y coordinate 
                return tmpPixel;//return pixel on contour
            }
        }
    }
return tmpPixel; //return default pixel

}


//return if pixel is on countour of potential field
bool checkPixel(std::vector<Pixel>& tmpWorklist, Pixel tmpPixel, Blob& blob, float threshold) {
    
    //calculate 4 points around pixel
    float PXminYmin   = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) - 0.5); 
    float PXminYplus  = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) + 0.5);
    float PXplusYmin  = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) - 0.5);
    float PXplusYplus = blob.potential(float(tmpPixel.x) + 0.5, float(tmpPixel.y) + 0.5);


    //make array of the four values
    std::array<float, 4> tmpArray{ PXminYmin,PXminYplus,PXplusYmin,PXplusYplus };
    //find max and min of value
    float max = *std::max_element(tmpArray.begin(), tmpArray.end());
    float min = *std::min_element(tmpArray.begin(), tmpArray.end());    

    if (max > threshold && min < threshold) { //if max and min are more and less than threshold repectively
        return true;
    }
    else{
        return false;
    }
}

//This function adds neighbour pixels to a worklist
void addNeighbourPixels(Pixel tmpPixel, std::vector<Pixel>& tmpWorklist) {
    Pixel curPixel; 
    for (int x = -1; x < 2; ++x) //loop of x coordinates
    {
        for (int y = -1; y < 2; ++y) //loop over y coordinates
        {
            if (x != 0 || y != 0) { //all 9 pixels exept the middle (0,0)
                curPixel.x = tmpPixel.x + x; //add current x plus offset
                curPixel.y = tmpPixel.y + y; // add current y plus offset
                tmpWorklist.push_back(curPixel); // add pixel to worklist
            }
        }
    }
}

//Search for threshold pixel by scanning, but it can search in reverse direction and skip lines to speed things up and find both vector fields(or twice the same)
Pixel searchThresholdPixelBetter(UI& ui, Blob& blob, float threshold, bool reverseScan)
{
    const int sizeX = ui.sizeX; //screen dimensions
    const int sizeY = ui.sizeY; //screen dimensions


    Pixel tmpPixel; //tmp pixel to work with


    if (!reverseScan) //scan normally when reversescan is false
    {
        for (int y = -sizeY / 2; y < sizeY / 2; y+=SKIP_NUM_ROWS) //loop through rows (y coordinates)
        {
            for (int x = -sizeX / 2; x < sizeX / 2; x++) //loop through colums (x coordinates)
            {
                if (blob.potential((float)x, (float)y) > threshold)// threshold found
                {
                    tmpPixel.x = x; 
                    tmpPixel.y = y; 
                    return tmpPixel; 
                }
            }
        }
    }
    else {
        for (int y = sizeY / 2; y > -sizeY / 2; y-=SKIP_NUM_ROWS) //loop through rows (y coordinates)
        {
            for (int x = sizeX / 2; x > -sizeX / 2; x--) //loop through rows (x coordinates)
            {
                if (blob.potential((float)x, (float)y) > threshold)
                {
                    tmpPixel.x = x;
                    tmpPixel.y = y;
                    return tmpPixel;
                }
            }
        }

    }
    return tmpPixel;
}

void fillBufferWithdata(Blob& blob, bool* pixels, float threshold, int totalThreads, int threadNumber) {

    
    Point curPoint;  //previous point
    Pixel curPixel; // current pixel

    const int sizeX = 1000; 
    const int sizeY = 600;
    const int offset = (sizeX * sizeY) / 2; // offset since screen middle is 0,0
    

    curPoint.x = (float)-sizeX/2; //first point to check against x coordinate
    curPoint.y = (float)-sizeY/2 + threadNumber * sizeY / totalThreads; //first point to check against y coordinate

    bool curPotOverThres;   //current potential of point                     
    bool prevPotOverThres;  //potential of previous point

    prevPotOverThres = blob.potential(curPoint) > threshold;    //set potential of first point            




    for (int y = -sizeY / 2 + 1 +threadNumber*sizeY/ totalThreads; y < -sizeY / 2+ (threadNumber+1) * sizeY / totalThreads; y++)  //loop through columns (y axis)
    {
        for (int x = -sizeX / 2; x < sizeX / 2; x++)    //loop through colums (x axis)
        {
            curPoint.x = (float)x;  //set x of current point
            curPoint.y = (float)y;   //set y of current point

            curPotOverThres = blob.potential(curPoint) > threshold;   //find current potential


            if (curPotOverThres != prevPotOverThres)    //check if current point is bigger than previous and bigger than threshold    
            {
                pixels[x + sizeX * y + offset] = true;  //set value of pixel in buffer
            }

            prevPotOverThres = curPotOverThres; //set new prev potential                
        }
    }

    

    
}

