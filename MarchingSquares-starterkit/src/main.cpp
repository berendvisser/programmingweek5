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
#include <map>
#include <cmath>
#include <array>




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

/// Scans full screen area. Complexity?
void drawContourScanning(UI &ui, Blob &blob, float threshold = 0.015)
{
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;

    // YOUR CODE HERE
    std::cout << "running Scanning\n";
    Point curPoint;                                         //previous point

    curPoint.x = (float)-sizeX;                             //first point to check against x coordinate
    curPoint.y = (float)-sizeY;                             //first point to check against y coordinate

    bool curPotOverThres;                                           //current potential of point                     
    bool prevPotOverThres;                                          //potential of previous point
    
    prevPotOverThres = blob.potential(curPoint) > threshold;                     //set potential of first point            
    

    ui.setDrawColor(255, 255, 255, 0);                      //set color to white

    for (int y = -sizeX+1; y < sizeX; y++) {                //loop through rows (y coordinates)
        for (int x = -sizeY; x < sizeY; x++) {              //loop through colums (x coordinates)
            curPoint.x = (float)x;                          //set x of current point
            curPoint.y = (float)y;                          //set y of current point
                      
            curPotOverThres = blob.potential(curPoint)>threshold;              //find current potential
            
            
            if (curPotOverThres != prevPotOverThres) {           //check if current point is bigger than previous and bigger than threshold                
                ui.drawPixel((int)curPoint.x, (int)curPoint.y);      //draw blob
            }

            prevPotOverThres = curPotOverThres;                                        //set net prev potential                
        }
    } 
}

/// Scans full screen area multithreaded. Complexity?
void drawContourScanningThreaded(UI &ui, Blob &blob, float threshold = 0.015)
{
    // YOUR CODE HERE

    
}

/// Scans screen area until it finds a pixel on the edge.
/// From then on, uses marching squares algorithm to find other pixels on the edge.
/// Caveat: only draws one of the potentially more than one contour losed loops.
/// Complexity?
void drawContourMarching(UI &ui, Blob &blob, float threshold = 0.015)
{
    // YOUR CODE HERE
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;
    const int offset = (sizeX * sizeY) / 2;

    std::cout << "running Marching\n";

    std::vector<Pixel> workList;    //Make worklist a vector

    bool* visitedPixels; 
    visitedPixels = new bool[sizeX*sizeY];

    for (int i = 0; i < sizeX * sizeY; i++) {
        visitedPixels[i] = false;
    }
    

    workList.push_back(searchThresholdPixel(ui, blob, threshold));  //Find pixel on curve and put on worklist
    

    while (!workList.empty()) {
        

        Pixel currentPixel = workList.back();  //find last pixel worklist and remove it
        workList.pop_back();
        bool x = visitedPixels[currentPixel.x + sizeX * currentPixel.y + offset];

        if (visitedPixels[currentPixel.x + sizeX*currentPixel.y+offset]) {     //Check if pixel is already visited
            
            continue; //skip to next iteration of loop
        }
        else {
            
            visitedPixels[currentPixel.x + sizeX * currentPixel.y + offset] = true; //Add pixel to list;
        }
        if (checkPixel(workList, currentPixel, blob, threshold)) {
            addNeighbourPixels(currentPixel, workList);
            ui.drawPixel(currentPixel.x, currentPixel.y);
        }
    } 
    delete visitedPixels;
}

/// Improved marching squares algorithm.
/// Should have better complexity than the initial algorithm described in the manual :)
void drawContourMarchingBetter(UI &ui, Blob &blob, float threshold = 0.015)
{
    // YOUR CODE HERE
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
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;
    Pixel tmpPixel;

    for (int y = -sizeX + 1; y < sizeX; y++) //loop through rows (y coordinates)
    {
        for (int x = -sizeY; x < sizeY; x = x + 30) //loop through colums (x coordinates)
        {
            if (blob.potential((float)x, (float)y) > threshold)
            {

                tmpPixel.x = x;
                tmpPixel.y = y;
                return tmpPixel;
            }
        }
    }

    

    for (int y = -sizeX + 1; y < sizeX; y++) //loop through rows (y coordinates)
    {
        for (int x = -sizeY; x < sizeY; x++) //loop through colums (x coordinates)
        {
            if (blob.potential((float)x, (float)y) > threshold) 
            {
                
                tmpPixel.x = x;
                tmpPixel.y = y;
                return tmpPixel;
            }
        }
    }
    return tmpPixel;
}


//return max potential around pixel
bool checkPixel(std::vector<Pixel>& tmpWorklist, Pixel tmpPixel, Blob& blob, float threshold) {
    
    float PXminYmin   = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) - 0.5);
    float PXminYplus  = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) + 0.5);
    float PXplusYmin  = blob.potential(float(tmpPixel.x) - 0.5, float(tmpPixel.y) - 0.5);
    float PXplusYplus = blob.potential(float(tmpPixel.x) + 0.5, float(tmpPixel.y) + 0.5);



    std::array<float, 4> tmpArray{ PXminYmin,PXminYplus,PXplusYmin,PXplusYplus };

    float max = *std::max_element(tmpArray.begin(), tmpArray.end());
    float min = *std::min_element(tmpArray.begin(), tmpArray.end());    

    if (max > threshold && min < threshold) {
        
        return true;
    }
    else{
        return false;
    }
}

void addNeighbourPixels(Pixel tmpPixel, std::vector<Pixel>& tmpWorklist) {
    Pixel curPixel;
    for (int x = -1; x < 2; ++x) {
        for (int y = -1; y < 2; ++y) {
            if (x != 0 || y != 0) {
                curPixel.x = tmpPixel.x + x;
                curPixel.y = tmpPixel.y + y;
                tmpWorklist.push_back(curPixel);
            }
        }
    }
}