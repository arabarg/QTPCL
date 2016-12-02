#include "viewer.h"
#include "../build/ui_viewer.h"
#include <iostream>

Viewer::Viewer(QWidget *parent) :  QWidget(parent),ui(new Ui::Viewer),
  filtering_axis_ (1), //y
  color_mode_ (1), //rainbow
  text_id (1),
  line_id (1)
{
    ui->setupUi(this);



    /**********************
     *
     *  Create a plane VTK
     *  Tampoco soy capaz de moverlo con VTK: Hay que crear un interactorstyle
     *  customizado juntando haciendo que el interactor style de pcl
     *  extienda de vtkInteractorStyleTrackballActor pero que añada la función de
     * vtkInteractorStyleRubberBandPick.
     * */
    vtkSmartPointer<interactor_style_actor> style =
        vtkSmartPointer<interactor_style_actor>::New();

   /*Creamos una nueva de puntos la rellenamos de puntos y la pintamos*/
    cloud_.reset(new PointCloudT);
    fillCloud();
    colorCloudDistances ();

    /*Creamos un visualizador pcl le añadimos los ojes de coordenadas y la nube de puntos*/
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_->addCoordinateSystem (1.0);
    viewer_->addPointCloud (cloud_, "cloud");

    /*Registramos los eventos en el visualizador para raton teclado seleccion y punteo*/
    viewer_->registerMouseCallback (&Viewer::mouseEventOccurred, *this, (void*)&viewer_);
    viewer_->registerPointPickingCallback(&Viewer::pickEventLine, *this, (void*)&viewer_);
    viewer_->registerKeyboardCallback (&Viewer::keyBoardEventOccurred,*this, (void*)&viewer_);
    viewer_->registerAreaPickingCallback(&Viewer::area_picking_callback, *this, (void*)&viewer_);

    /*Añadimos el visualizador al QVTKWidget de nuestra interfaz para poder verlo y manejarlo*/
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow (), style);

    /*Conectamos al interfaz con sus métodos: SIGNALS Y SLOTS DEL FORMULARIO*/
    connect (ui->free, SIGNAL(clicked ()), this, SLOT(addSquare()));
    connect (ui->line, SIGNAL(clicked ()), this, SLOT(addLine()));
    connect (ui->circle, SIGNAL(clicked ()), this, SLOT(addPlane()));
    connect (ui->load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
    connect (ui->save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));
    connect (ui->back, SIGNAL(clicked ()), this, SLOT(shootScreen()));
    connect (ui->rubber, SIGNAL(clicked()), this, SLOT(deleteDistances()));
    connect(ui->fullScreen, SIGNAL(clicked()), this, SLOT(showFullScreen()));
    connect(ui->verticalSlider, SIGNAL(valueChanged(int)), this, SLOT(fVoxelGrid(int)));



}

/*Eventos de teclado*/
void Viewer::keyBoardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){

/*NO FUNCIONA EL FULLSCREEN SE CUELGA*/

//    std::cout<<"Ha pulsado la letra: "<<event.getKeySym()<<endl;

//  if(event.getKeySym()=="Escape" && event.keyDown ()){
//      std::cout<<"presione esc";

//          viewer_->setFullScreen(false);
//          //this->setWindowState(Qt::WindowMaximized);
//      }
//  if(event.getKeySym()=="m" && event.keyDown ()){
//      std::cout<<"presione esc";

//          viewer_->setFullScreen(true);
//        //  this->setWindowState(Qt::WindowMaximized);
//      }

/**************************PRESS R************************/
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer_->removeShape (str);
    }
    text_id = 0;
  }
/*************************PRESS S*************************/
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "s was pressed => removing all lines" << std::endl;

    char line[512];
    for (unsigned int i = 0; i < line_id; ++i)
    {
      sprintf (line, "line#%03d", i);
      viewer_->removeShape (line);
    }
    line_id = 0;
  }
}
/*Eventos de ratón*/
void Viewer::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void){
     if(number_points!=0){
       if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
           event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
       {
         char str[512];
         char buffer[512];
         sprintf (str, "text#%03d", text_id++);
         sprintf (buffer, "(%d,%d)", event.getX(), event.getY());
         viewer_->addText (buffer, event.getX(), event.getY(),str);
       }
        }
 }
/*Eventos pick*/
void Viewer::pickEventLine (const pcl::visualization::PointPickingEvent& event, void* viewer_void){

 int idx = event.getPointIndex ();
 if (idx == -1)return;

 number_points++;

 if(number_points<=2){
    char str[512];
    char buffer[512];
    char line[512];
    char marker[512];
    char mesh[512];
    int i=1;
    float distance;
    PointT picked_point;
    PointT p1, p2;

    /*Detect point and save*/
    event.getPoint(picked_point.x,picked_point.y,picked_point.z);
    m_registration_points[number_points-1]=picked_point;

        /*Point Marker*/
        sprintf(marker, "sphere#%03d", marker_id++);
        viewer_->addSphere (picked_point, 0.01, 1.0, 0.0, 0.0, marker);
        if(number_points==2){
            /*Line*/
            sprintf(line,"line#%03d",line_id++ );
            viewer_->addLine(m_registration_points[0],m_registration_points[1],line);
            /*Distance*/
            distance=euclideanDistance(m_registration_points[0],m_registration_points[1]);
            sprintf (mesh, "mesh#%03d", mesh_id++);
            sprintf (buffer, "(d%i= %f m.)",mesh_id, distance);
            /*Combo*/

            ui->listWidget->addItem(buffer);
            ui->qvtkWidget->update ();
           }
      }

 }


/**** NO FUNCIONA BIEN**/

void Viewer::area_picking_callback (const pcl::visualization::AreaPickingEvent& event, void* viewer_void){
std::vector<int> indices;
    PointCloudT::Ptr cloud_pick;
       PointT current_point;
       cloud_pick.reset(new PointCloudT);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red (cloud_pick, 255, 0, 0);
      if (event.getPointsIndices(indices))
      {
          for(int i=0; i<indices.size();i++){
            current_point=cloud_->points[indices[i]];
            cloud_pick->points.push_back(current_point);
          }
          viewer_->removePointCloud("cloud_pick");
          viewer_->addPointCloud(cloud_pick, red, "cloud_pick");
          viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_pick");
        std::cout << "picked " << indices.size () <<std::endl;
         ui->qvtkWidget->update ();
      }
      else
        std::cout << "No valid points selected!" << std::endl;
 }


/*Rellena una nube de puntos y la muestra mientras no se sube ningun archivo*/
void Viewer::fillCloud(){

    cloud_->resize (500);

      // Fill the cloud with random points
      for (size_t i = 0; i < cloud_->points.size (); ++i)
      {
        cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }

}
void Viewer::colorCloudDistances ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  // Set an initial value
  switch (filtering_axis_)
  {
    case 0:  // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1:  // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default:  // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }

  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    switch (filtering_axis_)
    {
      case 0:  // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1:  // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default:  // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
}
void Viewer::addPlane(){

    pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);
    plane_1->values.resize (4);
    plane_1->values[0] = 0;
    plane_1->values[1] = 0;
    plane_1->values[2] = 1;
    plane_1->values[3] = 0;
    viewer_->addPlane (*plane_1, "plane_1", 0);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane_1", 0);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane_1", 0);

     ui->qvtkWidget->update ();
}


/*************************SLOTS***********************/

/*MENU INFERIOR*/
void Viewer::shootScreen(){
      originalPixmap = QPixmap::grabWindow(QApplication::desktop()->winId());
      saveScreenshot();
}
void Viewer::showFullScreen(){
    if(this->isFullScreen()) {
        //viewer_->setFullScreen(false);
        this->setWindowState(Qt::WindowMaximized);
    }
    else{
        this->setWindowState(Qt::WindowFullScreen);
        //viewer_->setFullScreen 	( 	true	);
    }

}
void Viewer::loadFileButtonPressed (){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));
          PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  PointCloudT::Ptr cloud_aux (new PointCloudT);
  PointCloudI::Ptr cloud_bi (new PointCloudI);
 PointCloudI::Ptr cloud_bi_filt (new PointCloudI);
  if (filename.isEmpty ()) return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

/*
 *FILTER: REMOVENANFROMPOINTCLOUD
*/
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  /*
   * FILTER VOXELGRID
   * */
  ui->verticalSlider->setValue(50);
 pcl::copyPointCloud (*cloud_,*cloud_aux);
  pcl::VoxelGrid<PointT> sor;
   sor.setInputCloud (cloud_);
   sor.setLeafSize (0.5f, 0.5f, 0.5f);
   sor.filter (*cloud_filtered);
   pcl::copyPointCloud (*cloud_filtered, *cloud_);

   /*
    * FILTER BILATERAL:: NO FUNCIONA NO SE SI ES POR EL TAMAÑO DE
    * LA IMAGEN
    * */
// [...]
//copyPointCloud(*cloud_,*cloud_bi);

//   pcl::BilateralFilter<PointI> BF;
//  pcl::search::KdTree<PointI>::Ptr tree (new     pcl::search::KdTree<PointI>);

//  BF.setSearchMethod(tree);
//  BF.setHalfSize(0.5);//el tamaño medio de la ventana de filtro bilateral de Gauss para usar.
//  BF.setStdDev (0.01);//el nuevo parámetro de desviación estándar.
//  BF.setInputCloud(cloud_bi);

//  BF.filter(*cloud_bi_filt);

// copyPointCloud(*cloud_bi_filt, *cloud_);


   /*
    * FILTER GRIDMINIMUN
    * */
//  pcl::GridMinimum <pcl::PointXYZRGBA> GridMini(5.2);
//           GridMini.setResolution(5.2);
//           GridMini.setInputCloud(cloud_);
//           GridMini.filter(*cloud_filtered);

//   pcl::copyPointCloud (*cloud_filtered, *cloud_);

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
   pcl::copyPointCloud (*cloud_aux, *cloud_);
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}
void Viewer::saveFileButtonPressed (){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}
void Viewer::saveScreenshot(){
    //Viewer* v;
    QString format = "png";

       QString initialPath = QDir::currentPath() + tr("/untitled.") + format;

//QString fileName = QFileDialog::getSaveFileName(this, tr ("Save As"),
             //                                   "/home/",
               //                                 tr ("(*.png)"));

QString fileName = QFileDialog::getSaveFileName(this, tr("Save As"),
                           initialPath,
                           tr("%1 Files (*.%2);;All Files (*)")
                           .arg(QString(format.toUpper()))
                           .arg(QString(format)));

       if (!fileName.isEmpty())
           originalPixmap.save(fileName, format.toLatin1());


}

/*TOOLBAR*/
void Viewer::deleteDistances(){
    /*delete lines*/
    char line[512];
    for (unsigned int i = 0; i < line_id; ++i)
    {
      sprintf (line, "line#%03d", i);
      viewer_->removeShape (line);
    }
    line_id = 0;
    /*delete markers*/
    char marker[512];
    for (unsigned int i = 0; i < marker_id; ++i)
    {
      sprintf (marker, "sphere#%03d", i);
      viewer_->removeShape (marker);
    }
    marker_id = 0;
    /*delete distances*/
   ui->listWidget->clear();
    marker_id = 0;
    /*delete free forms*/

    /*delete areas*/

    /*delete angles*/


    /*clean*/
 ui->qvtkWidget->update ();



}
void Viewer::addSquare(){

    PointT p1, p2, p3, p4;
  p1.x=0.0; p1.y=0.0; p1.z=0.0;
          p2.x=0.0; p2.y=4.0; p2.z=0.0;
            p3.x=4.0; p3.y=4.0; p3.z=0.0;
              p4.x=4.0; p4.y=0.0; p4.z=0.0;
     viewer_->addLine(p1,p2,"lin2");
     viewer_->addLine(p2,p3,"lin3");
viewer_->addLine(p3,p4,"lin4");
viewer_->addLine(p4,p1,"lin5");
 ui->qvtkWidget->update ();
}
void Viewer::addSphere(){
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize (3);    // We need 3 values
    circle_coeff.values[0] = 1.0;
    circle_coeff.values[1] = 1.0;
    circle_coeff.values[2] = 10;

    viewer_->addCircle (circle_coeff,  "sphere");
     ui->qvtkWidget->update ();
}
void Viewer::addLine(){
 std::cout<<"addLine"<<std::endl;
number_points=0;
 }


/*FILTRADO PCL*/
void Viewer::fVoxelGrid(int value){
PointCloudT::Ptr cloud_filtered (new PointCloudT);
PointCloudT::Ptr cloud_aux (new PointCloudT);

 pcl::copyPointCloud (*cloud_,*cloud_aux);
    pcl::VoxelGrid<PointT> sor;
     sor.setInputCloud (cloud_);
     float realvalue=0.01*value;
     sor.setLeafSize (realvalue,realvalue,realvalue);
     sor.filter (*cloud_filtered);
   pcl::copyPointCloud (*cloud_filtered, *cloud_);

   colorCloudDistances ();
    viewer_->updatePointCloud (cloud_, "cloud");
     pcl::copyPointCloud (*cloud_aux, *cloud_);


    ui->qvtkWidget->update ();



}





Viewer::~Viewer()
{
    delete ui;
}
