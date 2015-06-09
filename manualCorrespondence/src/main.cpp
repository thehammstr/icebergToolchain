#include "corrGUI.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  if (argc < 2 ){
    std::cout<<"Usage: ./correspondenceGUI [filename]\n";
    return 0;
  }
  

  QApplication a (argc, argv);
  PCLViewer w; 
  std::cout<<argv[1]<<std::endl;
  if (argc < 3){
      w.loadTrajectory(argv[1]);
  } else{
      w.loadTrajectory(argv[1],argv[2]);
  }
  w.show ();

  return a.exec ();
}
