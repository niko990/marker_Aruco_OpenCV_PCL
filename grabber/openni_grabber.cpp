#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/point_types.h>


bool save_one =false;
bool one_time =true;
int capture =0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void){
if (event.getKeySym () == "s" && event.keyDown ()){
std::cout<<"Now saving... please wait" <<std::endl;
save_one =true;
}
}

void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
if(one_time==true){
   viewer.setBackgroundColor (1.0, 0.5, 1.0);
   std::cout << "FRAME GRABBER with pcl" << std::endl;
std::cout << "All you have to do to save a cloud is press S key" << std::endl;
viewer.setCameraPosition(0,0,-2,0,-1,0,0);
viewer.resetCamera();
one_time=false;
}
   
}


class SimpleOpenNIViewer
{
  public:
   SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);

   if( save_one) { 
                               save_one = false;
std::stringstream out; 
std::cout << "Cloud saved: capture" + capture << std::endl;
                          
frames_saved++; 
                               std::stringstream name;
if (capture<10){name<<"capture00"<<capture<<".pcd";}else{
if (capture<100){name<<"capture0"<<capture<<".pcd";}else{
name<<"capture"<<capture<<".pcd";}}	
pcl::io::savePCDFileASCII( name.str(), *cloud );
capture++;
                       } 

    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();
viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);


      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
viewer.runOnVisualizationThreadOnce (viewerOneOff);
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;

      
private: 
               int frames_saved; 

};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}