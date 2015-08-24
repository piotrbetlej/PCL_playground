#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <iostream>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

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

static void create_random_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (size_t var = 0; var < cloud->points.size(); ++var) {
        float theta = 2*(3.14159276f)*(rand() / ((float)RAND_MAX));
        float phi = (3.14159276f)*(rand() / ((float)RAND_MAX));
        float r = rand() % 5;

        cloud->points[var].x = r*cos(theta)*sin(phi);
        cloud->points[var].y= r*sin(theta)*sin(phi);
        cloud->points[var].z = r*cos(phi);
    }
}

void extract_borders( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,
                                   pcl::PointCloud<pcl::BorderDescription>::Ptr borders_p)
{
    // Convert the cloud to range image.
        int imageSizeX = 640, imageSizeY = 480;
        float centerX = (640.0f / 2.0f), centerY = (480.0f / 2.0f);
        float focalLengthX = 0.01f, focalLengthY = focalLengthX;
        Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud_p->sensor_origin_[0],
                                     cloud_p->sensor_origin_[1],
                                     cloud_p->sensor_origin_[2])) *
                                     Eigen::Affine3f(cloud_p->sensor_orientation_);
        float noiseLevel = 0.0f, minimumRange = 0.0f;
        pcl::RangeImagePlanar rangeImage;
        rangeImage.createFromPointCloudWithFixedSize(*cloud_p, imageSizeX, imageSizeY,
                centerX, centerY, focalLengthX, focalLengthX,
                sensorPose, pcl::RangeImage::CAMERA_FRAME,
                noiseLevel, minimumRange);

        // Border extractor object.
        pcl::RangeImageBorderExtractor borderExtractor(&rangeImage);

        borderExtractor.compute(*borders_p);
        // Visualize the borders.
        pcl::visualization::RangeImageVisualizer* riviewer = NULL;
        riviewer=pcl::visualization::RangeImageVisualizer::getRangeImageWidget(rangeImage, -std::numeric_limits<float>::infinity(),
                                                                               std::numeric_limits<float>::infinity(),
                                                                               false,"Range Image");
//        riviewer = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(rangeImage,
//                                                                                        -std::numeric_limits<float>::infinity(),
//                                                                                        std::numeric_limits<float>::infinity(),
//                                                                                        false, *borders_p, "Borders");

}
#ifdef ACTIVE
int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Object for storing the borders.
        pcl::PointCloud<pcl::BorderDescription>::Ptr borders(new pcl::PointCloud<pcl::BorderDescription>);

    string filename;
    if (console::find_argument(argc, argv, "-v") >= 0)
    {
        filename = argv[2];
        io::loadPCDFile<pcl::PointXYZ>(filename.c_str(), *cloud);
    }

    int a = 0;
    int b = 1;

    extract_borders(cloud,borders);
    viewer = createViewer();

    viewer->createViewPort (0.0,0.0,0.5,1.0,a);
    viewer->setBackgroundColor(0,0,0,a); // background color dark
    viewer->addCoordinateSystem(1,"CoordinateSystem",a);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud",a);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}
#endif
