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
  w.loadTrajectory(argv[1]);
  w.show ();

  return a.exec ();
}
