#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

using namespace pcl;

static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

static void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown());
}


static void pointPickingEventHandler(const pcl::visualization::PointPickingEvent e){
    float x,y,z;
    e.getPoint(x,y,z);
    PointXYZ p2 =  PointXYZ(x,y,z);
    PointXYZ p1 = PointXYZ(x+100,y+100,z+100);

    // Get random number as string to number arrows.
    // Arrows must have different names.
    int random_number_arrow = rand() % 1000;
    std::ostringstream oss_arrow;
    oss_arrow << random_number_arrow;

    // Get random number as string to number arrows.
    // Arrows must have different names.
    int random_number_text = rand() % 1000;
    std::ostringstream oss_text;
    oss_text << random_number_text;

    // Add text nearby marked point.

    std::ostringstream oss2;
    oss2 << "X: " << x << "Y: " <<  y << "Z: " << z ;

    // Adds an arrow between points p1 and p2 and distance between them.
    viewer->addArrow(p1,p2,0.5, 0.5, 0.0,oss_arrow.str());
    viewer->addText3D(oss2.str(),p2,1.0,1.0,1.0,1.0,oss_text.str());
}

static boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> v
            (new pcl::visualization::PCLVisualizer(""));

    v->registerKeyboardCallback(keyboardEventOccurred);
    v->registerPointPickingCallback(pointPickingEventHandler);
    return (v);
}


void create_random_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (size_t var = 0; var < cloud->points.size(); ++var) {
        float theta = 2*(3.14159276f)*(rand() / ((float)RAND_MAX));
        float phi = (3.14159276f)*(rand() / ((float)RAND_MAX));
        float r = rand() % 100;

        cloud->points[var].x = r*cos(theta)*sin(phi);
        cloud->points[var].y= r*sin(theta)*sin(phi);
        cloud->points[var].z = r*cos(phi);
    }
}

#ifdef ACTIVE
int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 100;
    cloud->height = 100;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width*cloud->height);

    // Create spherical cloud with random points
    create_random_sphere(cloud);


//    for (size_t var = 0; var < cloud->points.size(); ++var) {
//        cloud->points[var].x = 1024*rand() / (RAND_MAX + 1.0f);
//        cloud->points[var].y= 1024*rand() / (RAND_MAX + 1.0f);
//        cloud->points[var].z = 1024*rand() / (RAND_MAX + 1.0f);
//    }

    viewer = createViewer();
    viewer->addCoordinateSystem(1,"CoordinateSystem");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    int a = 0;
    int b = 1;

    viewer->createViewPort (0.0,0.0,0.5,1.0,a);
    viewer->setBackgroundColor(0,0,0,a); // background color dark
    viewer->addText("sample_cloud_1", 10, 10, "right", a);
    viewer->addPointCloud(cloud,  "sample_cloud_1", a);

    viewer->createViewPort (0.5,0.0,0.5,1.0,b);
    viewer->setBackgroundColor(0.1,0.1,0.1,b); // background color light
    viewer->addText("sample_cloud_2", 10, 10, "left", b);
    viewer->addPointCloud(cloud,  "sample_cloud_2", b);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}
#endif
