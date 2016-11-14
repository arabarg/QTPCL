#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>


// Qt
#include <QWidget>
#include <QFileDialog>
#include <QIcon>
#include <QPixmap>
#include <QDesktopWidget>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/grid_minimum.h>

//Eigen

#include <eigen3/Eigen/Geometry>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
namespace Ui {
class Viewer;
}

class Viewer : public QWidget
{
    Q_OBJECT

public:
    explicit Viewer(QWidget *parent = 0);
    ~Viewer();
void saveScreenshot();
private:
    Ui::Viewer *ui;

protected:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    int text_id=0;
    int line_id=0;
    int marker_id=0;
    int mesh_id=0;
    PointCloudT::Ptr cloud_;
    PointT p1,p2;
    PointT m_registration_points[2];
    QPixmap originalPixmap;
    int filtering_axis_;
    int color_mode_;
    int number_points;
    void  colorCloudDistances ();
    void fillCloud();
    void addPlane();
    void mouseEventOccurred (const pcl::visualization::MouseEvent& event, void* viewer_void);
    void keyBoardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                void* viewer_void);
    void mouseEventLine (const pcl::visualization::MouseEvent &event, void* viewer_void);
    void pickEventLine (const pcl::visualization::PointPickingEvent &event, void* viewer_void);
    void area_picking_callback (const pcl::visualization::AreaPickingEvent &event, void*);
public slots:
    void shootScreen();
    void saveFileButtonPressed ();
    void loadFileButtonPressed ();
    void addLine();
    void addSquare();
    void addSphere();
    void deleteDistances();
void showFullScreen();
void fVoxelGrid(int value);



};

#endif // VIEWER_H
