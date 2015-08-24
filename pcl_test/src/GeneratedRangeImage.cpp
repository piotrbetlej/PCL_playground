#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>

using namespace std;
using namespace pcl;
//----------Working gneneration coordinates-------------------
//                 minx maxx miny maxy minz maxz step camposx camposy camposy camresx camresy
// -v aaa.pcl 5 15 -5.0 5.0 -2.0 2.0 0.05 10.0 0.0 0.0 640 480
// -v test.pcl -1.0 1.0 -1.0 1.0 10.0 10.1 0.01 0.0 0.0 0.0 640 480 (focal length 525)

// Focal length shall be big enaugh for planar range image.
// Sensor pose [0,0,0] is pointing along OZ from point 0 to +infinity.

static void generate_ri(RangeImage& sphe_ri, RangeImagePlanar& plan_ri, pcl::PointCloud<pcl::PointXYZ>::Ptr* cloud,
                        float xmin,float xmax,float ymin,float ymax,float zmin,float zmax, float step,
                        float plan_posex,float plan_posey,float plan_posez, float imageSizeX, float imageSizeY){

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

//    float xmin,xmax;
//    xmin = -1.0f; xmax = -5.0f;
//    float ymin,ymax;
//    ymin = -5.0f;  ymax =  5.0f;
//    float zmin,zmax;
//    zmin = -1.0f , zmax = 1.0f;
//    float step;
//    step = 0.01f;

    // Generate the data
    for (float x=xmin; x<=xmax; x+=step) {
        for (float y=ymin; y<=ymax; y+=step) {
            for (float z=zmin; z<=zmax; z+=step) {
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                pointCloud->points.push_back(point);
            }
        }
    }
    pointCloud->width = (uint32_t) pointCloud->points.size();
    pointCloud->height = 1;
    pointCloud->is_dense = true;

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  0.050f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(plan_posex,plan_posey, plan_posez);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage sphe_rangeImage;
    sphe_rangeImage.createFromPointCloud(*pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    sphe_rangeImage.copyTo(sphe_ri);

    // Ptr is a class. Use pointer to this class to return value.
    *cloud=(*pointCloud).makeShared();

    // Convert the cloud to planar range image.
    // int imageSizeX = 640, imageSizeY = 480;
    float centerX = (imageSizeX / 2.0f), centerY = (imageSizeY/ 2.0f);

    float focalLengthX = 525.0f, focalLengthY = focalLengthX;
    float plan_noiseLevel = 0.0f, plan_minimumRange = 0.0f;
    pcl::RangeImagePlanar plan_rangeImage;

    plan_rangeImage.createFromPointCloudWithFixedSize(*pointCloud, imageSizeX, imageSizeY,
                                                 centerX, centerY, focalLengthX, focalLengthY,
                                                 sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                 plan_noiseLevel, plan_minimumRange);
    plan_rangeImage.copyTo(plan_ri);
}

//#define ACTIVE
#ifdef ACTIVE
int
main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    string filename;
    float xmin,xmax,ymin,ymax,zmin,zmax,step,
            plan_pose_x,plan_pose_y,plan_pose_z,
            imageSizeX,imageSizeY;
    xmin = atof(((string)argv[3]).c_str());
    xmax = atof(((string)argv[4]).c_str());
    ymin = atof(((string)argv[5]).c_str());
    ymax = atof(((string)argv[6]).c_str());
    zmin = atof(((string)argv[7]).c_str());
    zmax = atof(((string)argv[8]).c_str());
    step = atof(((string)argv[9]).c_str());
    plan_pose_x = atof(((string)argv[10]).c_str());
    plan_pose_y = atof(((string)argv[11]).c_str());
    plan_pose_z = atof(((string)argv[12]).c_str());
    imageSizeX = atof(((string)argv[13]).c_str());
    imageSizeY = atof(((string)argv[14]).c_str());

    if (console::find_argument(argc, argv, "-v") >= 0)
    {
        filename = argv[2];
        io::loadPCDFile<pcl::PointXYZ>(filename.c_str(), *cloud);
    }


    // Parameters needed by the planar range image object:

    // Image size. Both Kinect and Xtion work at 640x480.
    imageSizeX = 640;
    imageSizeY = 480;
    // Center of projection. here, we choose the middle of the image.
    float centerX = 640.0f / 2.0f;
    float centerY = 480.0f / 2.0f;
    // Focal length. The value seen here has been taken from the original depth images.
    // It is safe to use the same value vertically and horizontally.
    float focalLengthX = 525.000f, focalLengthY = focalLengthX;
    // Sensor pose. Thankfully, the cloud includes the data.
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                                 cloud->sensor_origin_[1], cloud->sensor_origin_[2])) * Eigen::Affine3f(cloud->sensor_orientation_);
    sensorPose = Eigen::Affine3f(Eigen::Translation3f(0.0f,
                                                      0.0f, 1.0f)) * Eigen::Affine3f(cloud->sensor_orientation_);

    // Noise level. If greater than 0, values of neighboring points will be averaged.
    // This would set the search radius (i.e., 0.03 == 3cm).
    float noiseLevel = 0.1f;
    // Minimum range. If set, any point closer to the sensor than this will be ignored.
    float minimumRange = 0.0f;

    // Planar range image object.
    //	pcl::RangeImagePlanar rangeImagePlanar;
    //	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
    //			centerX, centerY, focalLengthX, focalLengthX,
    //			sensorPose, pcl::RangeImage::CAMERA_FRAME,
    //            noiseLevel, minimumRange);

    // Angular resolution is the angular distance between pixels.
    // Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
    // Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
    float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
    float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
    // Maximum horizontal and vertical angles. For example, for a full panoramic scan,
    // the first would be 360º. Choosing values that adjust to the real sensor will
    // decrease the time it takes, but don't worry. If the values are bigger than
    // the real ones, the image will be automatically cropped to discard empty zones.
    float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
    // Sensor pose. Thankfully, the cloud includes the data.

    cloud->width = (uint32_t) cloud->points.size();
    cloud->height = 1;

    boost::shared_ptr<pcl::RangeImage> sphe_range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& sph_range_image = *sphe_range_image_ptr;

    boost::shared_ptr<pcl::RangeImagePlanar> plan_range_image_ptr(new pcl::RangeImagePlanar);
    pcl::RangeImagePlanar& plan_range_image = *plan_range_image_ptr;


    angularResolutionX = angularResolutionX / 2.0f;
    angularResolutionY = angularResolutionY / 2.0f;
    sph_range_image.createFromPointCloud (*cloud, angularResolutionX,angularResolutionY,
                                          pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                          sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minimumRange, 1);

    //*************************************************************************************************************************
    // Visualize the image.
    pcl::visualization::RangeImageVisualizer sphe_viewer("Spherical range image");
    pcl::visualization::RangeImageVisualizer plan_viewer("Planar range image");
    // viewer.showRangeImage(rangeImagePlanar);

    pcl::PointCloud<pcl::PointXYZ>::Ptr genercloud;

    // This cloud works.
    generate_ri(sph_range_image,plan_range_image,&genercloud,
                xmin,xmax,ymin,ymax,zmin,zmax,step,
                plan_pose_x,plan_pose_y,plan_pose_z,imageSizeX,imageSizeY);

    //------------------------
    //----Show cloud----
    //------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloudviewer
            (new pcl::visualization::PCLVisualizer(""));
    cloudviewer->addCoordinateSystem(1,"CoordinateSystem");
    cloudviewer->addPointCloud<pcl::PointXYZ>(genercloud, "Generated cloud");

    //----------------------------------------------
    //----Show spherical range image----
    //----------------------------------------------
    sphe_viewer.showRangeImage(sph_range_image);
    plan_viewer.showRangeImage(plan_range_image);

    while (!sphe_viewer.wasStopped())
    {
        sphe_viewer.spinOnce();
        plan_viewer.spinOnce();
        cloudviewer->spinOnce();
        // Sleep 100ms to go easy on the CPU.
        pcl_sleep(0.1);
    }
}
#endif

