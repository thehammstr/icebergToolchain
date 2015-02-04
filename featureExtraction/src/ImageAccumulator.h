#ifndef IMGACC
#define IMGACC



class PixelAccumulator {
  public:
    int Npoints;
    double height;
    PixelAccumulator() : Npoints(0), height(-17.){}
    void Accumulate(double);

};

inline void PixelAccumulator::Accumulate(double val){
  
  height = ((double)Npoints/(Npoints+1.))*height + (1./(Npoints+1.))*val;
  Npoints++;

  return;
}

#define MAX_GRID 2000
class ImageAccumulator {
  public:
    int height;
    int width;
    PixelAccumulator grid[MAX_GRID][MAX_GRID];
    ImageAccumulator(int h, int w);
    ~ImageAccumulator();
};







#endif
