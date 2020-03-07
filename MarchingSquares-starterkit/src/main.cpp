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
#include <iostream>
#include <cmath>

constexpr bool capFramerate = false;
//constexpr float thresholds[] = [0.015];
constexpr float thresholds[] = {0.015, 0.02, 0.03};



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

/// Scans full screen area. Complexity?
void drawContourScanning(UI &ui, Blob &blob, float threshold = 0.015)
{
    const int sizeX = ui.sizeX;
    const int sizeY = ui.sizeY;

    // YOUR CODE HERE
    Point curPoint;                                         //previous point

    curPoint.x = (float)-sizeX;                                    //first point to check against x coordinate
    curPoint.y = (float)-sizeY;                                    //first point to check against y coordinate

    float curPot;                                           //current potential of point                     
    float prevPot;                                          //potential of previous point
    
    prevPot = blob.potential(curPoint);                     //set potential of first point            
    

    ui.setDrawColor(255, 255, 255, 0);                      //set color to white

    for (int y = -sizeX+1; y < sizeX; y++) {                //loop through rows (y coordinates)
        for (int x = -sizeY; x < sizeY; x++) {              //loop through colums (x coordinates)
            curPoint.x = (float)x;                                 //set x of current point
            curPoint.y = (float)y;                                 //set y of current point
                      
            curPot = blob.potential(curPoint);              //find current potential
            
            
            if (curPot != prevPot && curPot > threshold) {           //check if current point is bigger than previous and bigger than threshold
                
                ui.drawPixel((int)curPoint.x, (int)curPoint.y);      //draw blob
            }
            prevPot = curPot;                                        //set net prev potential
                
            
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
