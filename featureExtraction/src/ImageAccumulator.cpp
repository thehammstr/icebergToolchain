#include "ImageAccumulator.h"

ImageAccumulator::ImageAccumulator(int h, int w){

  height = h;
  width = w;
  grid = new PixelAccumulator[MAX_GRID][MAX_GRID];

}

ImageAccumulator::~ImageAccumulator(){
  if(grid)
    delete grid;
}
