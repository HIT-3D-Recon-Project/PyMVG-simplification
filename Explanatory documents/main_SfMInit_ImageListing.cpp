// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/system/loggerprogress.hpp"
#include "openMVG/types.hpp"
//openMVG库头文件
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
//第三方库头文件
#include <fstream>
#include <memory>
#include <string>
#include <utility>
//标准库头文件
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;
//使用命名空间
/// 检查摄像机内参矩阵（K matrix）是不是 "f;0;ppx;0;f;ppy;0;0;1" 形式的字符串
/// 以及其中的 f,ppx,ppy 是否是有效数值
//然而，根据pipeline，没有输入sKmatrix，那好像不会触发这个。
bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
{
  std::vector<std::string> vec_str;
  stl::split(Kmatrix, ';', vec_str);
  if (vec_str.size() != 9)  {
    OPENMVG_LOG_ERROR << "\n Missing ';' character";
    return false;
  }
  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i) {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      OPENMVG_LOG_ERROR << "\n Used an invalid not a number character";
      return false;
    }
    if (i==0) focal = readvalue;
    if (i==2) ppx = readvalue;
    if (i==5) ppy = readvalue;
  }
  return true;
}
/*
    函数定义为bool getGPS，接受三个参数：filename（一个字符串，指定图片文件的路径），GPS_to_XYZ_method（一个整数，指定将GPS坐标转换到三维空间点的方法），以及pose_center（一个Vec3类型的引用，用于存储转换后的三维空间点）。

    首先，创建一个Exif_IO类型的智能指针exifReader，并使用Exif_IO_EasyExif构造函数初始化。这一步是为了读取图片文件中的EXIF信息，EXIF信息通常包含了拍照时的各种设置和条件，包括GPS坐标。

    接下来，通过if (exifReader)判断exifReader是否成功创建。

    在if语句内部，尝试打开指定的图片文件并检查该文件是否包含EXIF信息，通过调用exifReader->open(filename)和exifReader->doesHaveExifInfo()实现。

    如果文件成功打开并且包含EXIF信息，那么进一步检查是否存在GPS坐标信息。这是通过调用exifReader->GPSLatitude(&latitude), exifReader->GPSLongitude(&longitude)和exifReader->GPSAltitude(&altitude)实现的，分别获取纬度、经度和高度信息。

    如果成功获取到GPS坐标，根据GPS_to_XYZ_method的值，选择不同的转换方法将GPS坐标转换为三维空间中的点。case 1:是将GPS坐标转换为UTM（通用横轴墨卡托）坐标，case 0:和default:是将GPS坐标转换为ECEF（地球中心地固坐标系）坐标。转换后的坐标被赋值给pose_center。

    如果一切顺利，即GPS坐标被成功转换，函数返回true表示成功获取和转换GPS坐标;如果在任何步骤中出现失败（如无法打开文件、文件中不包含EXIF信息或不包含GPS信息），函数最终返回false，表示没有成功获取GPS坐标。
*/
//这个函数能不能出结果决定了三维向量pose_center的构造过程。我看了下，手机拍的照片有EXIF信息，但是没有经纬度。项目给的城堡实例也没有，所以这个getGPS函数就是个false返回器。
bool getGPS
(
  const std::string & filename,
  const int & GPS_to_XYZ_method,
  Vec3 & pose_center
)
{
  std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
  if (exifReader)
  {
    // Try to parse EXIF metada & check existence of EXIF data
    if ( exifReader->open( filename ) && exifReader->doesHaveExifInfo() )
    {
      // Check existence of GPS coordinates
      double latitude, longitude, altitude;
      if ( exifReader->GPSLatitude( &latitude ) &&
           exifReader->GPSLongitude( &longitude ) &&
           exifReader->GPSAltitude( &altitude ) )
      {
        // Add ECEF or UTM XYZ position to the GPS position array
        switch (GPS_to_XYZ_method)
        {
          case 1:
            pose_center = lla_to_utm( latitude, longitude, altitude );
            break;
          case 0:
          default:
            pose_center = lla_to_ecef( latitude, longitude, altitude );
            break;
        }
        return true;
      }
    }
  }
  return false;
}


/// 尝试将预设权重字符串sWeights打散成vec_str然后输入到返回值（布尔-向量对）里返回。虽然参数没输入，但是main里设了初始值，所以结果是<True,三个1.0组成的向量>。
std::pair<bool, Vec3> checkPriorWeightsString
(
  const std::string &sWeights
)
{
  std::pair<bool, Vec3> val(true, Vec3::Zero());
  std::vector<std::string> vec_str;
  stl::split(sWeights, ';', vec_str);
  if (vec_str.size() != 3)
  {
    OPENMVG_LOG_ERROR << "Missing ';' character in prior weights";
    val.first = false;
  }
  // Check that all weight values are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      OPENMVG_LOG_ERROR << "Used an invalid not a number character in local frame origin";
      val.first = false;
    }
    val.second[i] = readvalue;
  }
  return val;
}
//
// Create the description of an input image dataset for OpenMVG toolsuite
// - Export a SfM_Data file with View & Intrinsic data
// 重头戏来咯，输入图像（sImageDir里的）和相机数据库（sensor_width_camera_database.txt）；输出sfm_data.json包含图像尺寸，焦距（单位像素）
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sfileDatabase = "",
    sOutputDir = "",
    sKmatrix;

  std::string sPriorWeights = "1.0;1.0;1.0";
  std::pair<bool, Vec3> prior_w_info(false, Vec3());

  //枚举常量PINHOLE_CAMERA_RADIAL3默认值为3。从pipeline的参数来看也不打算变了。所以就是3而已。
  //根据神奇海螺和GitHub上的分析，3指的是含有“径向畸变K1,K2,K3”的相机模型，“这种模型用于描述和校正相机拍摄图像时产生的畸变，特别是在边缘区域。”
  //以及设了一些默认值：
  //b_Group_camera_model是一个布尔值，用于指示是否应该对使用相同相机模型的相机进行分组。如果设置为true，则意味着系统在处理多个图像时，会将使用相同模型的相机视为一组，这有助于优化处理流程和提高效率。
  //i_GPS_XYZ_method指的是将GPS坐标（经纬度和高度）转换为三维空间坐标（XYZ）的方法。这里的0可能表示使用了特定的转换方法，比如地球中心地固坐标系（ECEF）或其他。不同的转换方法可能适用于不同的应用场景和精度要求。
  //focal_pixels代表相机焦距的像素值。焦距是相机镜头特性的一个关键参数，对于图像处理和三维重建非常重要。这里的-1.0可能表示该值未被初始化或需要通过其他方式确定。在实际应用中，焦距可以从EXIF数据中读取，或者通过其他方法估计。
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
  
  bool b_Group_camera_model = true;//1-> (default) view can share some camera intrinsic parameters

  int i_GPS_XYZ_method = 0;

  double focal_pixels = -1.0;
  //然而，只用了前面三个参数，所以貌似没有什么sKmatrix了
  //这里代号为d的参数，调用了记录市面上各种常见相机“传感器尺寸”的sensor_width_camera_database.txt（像字典一样）以供查询。城堡图片集的相机型号也记录了，值是5.773，但是，我手机的相机型号在里面找不到。
  //也就是说，要么借相机（也有可能还是找不到），要么想办法搞清楚某台手机的“传感器尺寸”。
  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('d', sfileDatabase, "sensorWidthDatabase") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
  cmd.add( make_option('f', focal_pixels, "focal") );
  cmd.add( make_option('k', sKmatrix, "intrinsics") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );
  cmd.add( make_switch('P', "use_pose_prior") );
  cmd.add( make_option('W', sPriorWeights, "prior_weights"));
  cmd.add( make_option('m', i_GPS_XYZ_method, "gps_to_xyz_method") );
  //如果没输够参数，就报错并且弹出help教你怎么用。哈哈。
  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-d|--sensorWidthDatabase]\n"
      << "[-o|--outputDirectory]\n"
      << "[-f|--focal] (pixels)\n"
      << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
      << "[-c|--camera_model] Camera model type:\n"
      << "\t" << static_cast<int>(PINHOLE_CAMERA) << ": Pinhole\n"
      << "\t" << static_cast<int>(PINHOLE_CAMERA_RADIAL1) << ": Pinhole radial 1\n"
      << "\t" << static_cast<int>(PINHOLE_CAMERA_RADIAL3) << ": Pinhole radial 3 (default)\n"
      << "\t" << static_cast<int>(PINHOLE_CAMERA_BROWN) << ": Pinhole brown 2\n"
      << "\t" << static_cast<int>(PINHOLE_CAMERA_FISHEYE) << ": Pinhole with a simple Fish-eye distortion\n"
      << "\t" << static_cast<int>(CAMERA_SPHERICAL) << ": Spherical camera\n"
      << "[-g|--group_camera_model]\n"
      << "\t 0-> each view have it's own camera intrinsic parameters,\n"
      << "\t 1-> (default) view can share some camera intrinsic parameters\n"
      << "\n"
      << "[-P|--use_pose_prior] Use pose prior if GPS EXIF pose is available"
      << "[-W|--prior_weights] \"x;y;z;\" of weights for each dimension of the prior (default: 1.0)\n"
      << "[-m|--gps_to_xyz_method] XZY Coordinate system:\n"
      << "\t 0: ECEF (default)\n"
      << "\t 1: UTM";

      OPENMVG_LOG_ERROR << s;
      return EXIT_FAILURE;
  }
  //'P', "use_pose_prior"，你说用先验位姿就b_Use_pose_prior设为1
  const bool b_Use_pose_prior = cmd.used('P');
  //弹出信息供确认
  OPENMVG_LOG_INFO << " You called : " << argv[0]
    << "\n--imageDirectory " << sImageDir
    << "\n--sensorWidthDatabase " << sfileDatabase
    << "\n--outputDirectory " << sOutputDir
    << "\n--focal " << focal_pixels
    << "\n--intrinsics " << sKmatrix
    << "\n--camera_model " << i_User_camera_model
    << "\n--group_camera_model " << b_Group_camera_model
    << "\n--use_pose_prior " << b_Use_pose_prior
    << "\n--prior_weights " << sPriorWeights
    << "\n--gps_to_xyz_method " << i_GPS_XYZ_method;

  // 每张图片的具体参数
  double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;
  // 确认相机畸变模型？不知道在干什么
  const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);
  // 错误输入输出路径排查
  if ( !stlplus::folder_exists( sImageDir ) )
  {
    OPENMVG_LOG_ERROR << "The input directory doesn't exist";
    return EXIT_FAILURE;
  }

  if (sOutputDir.empty())
  {
    OPENMVG_LOG_ERROR << "Invalid output directory";
    return EXIT_FAILURE;
  }

  if ( !stlplus::folder_exists( sOutputDir ) )
  {
    if ( !stlplus::folder_create( sOutputDir ))
    {
      OPENMVG_LOG_ERROR << "Cannot create output directory";
      return EXIT_FAILURE;
    }
  }
  //检查sKmatrix合不合规格，但是默认值是空的，所以不检查。
  if (sKmatrix.size() > 0 &&
    !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy) )
  {
    OPENMVG_LOG_ERROR << "Invalid K matrix input";
    return EXIT_FAILURE;
  }
  //这k和f还不能一起给
  if (sKmatrix.size() > 0 && focal_pixels != -1.0)
  {
    OPENMVG_LOG_ERROR << "Cannot combine -f and -k options";
    return EXIT_FAILURE;
  }
  //如果不为空，则尝试使用parseDatabase函数解析这个数据库文件，并逐行将有效数据转换为Datasheet对象，填充到向量vec_database中。就是数据转移。
  std::vector<Datasheet> vec_database;
  if (!sfileDatabase.empty())
  {
    if ( !parseDatabase( sfileDatabase, vec_database ) )
    {
      OPENMVG_LOG_ERROR
       << "Invalid input database: " << sfileDatabase
       << ", please specify a valid file.";
      return EXIT_FAILURE;
    }
  }

  // 如果用户说b_Use_pose_prior，则调用checkPriorWeightsString函数处理字符串sPriorWeights（这个字符串应该包含用于位姿估计的先验权重，默认值111），并将结果（pair）存储在prior_w_info中。
  if (b_Use_pose_prior)
  {
    prior_w_info = checkPriorWeightsString(sPriorWeights);
  }
  //存图像名字到向量vec_image并排序
  std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
  std::sort(vec_image.begin(), vec_image.end());

  // SfM_Data实例出来啦！不过现在只有图像根目录作为属性储存
  // 声明对视图和内参的引用，不过我到现在还不知道引用功能的具体作用
  SfM_Data sfm_data;
  sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views; 
  Intrinsics & intrinsics = sfm_data.intrinsics;

  //实例化一个进度条my_progress_bar，用于跟踪和显示列出图像的进度。
  //创建一个std::ostringstream实例error_report_stream，用于收集过程中可能发生的错误信息。
  system::LoggerProgress my_progress_bar(vec_image.size(), "- Listing images -" );
  std::ostringstream error_report_stream;

  /*
    使用一个循环遍历vec_image向量中的每一个图像文件名。
    在循环内部，针对每个图像文件进行处理（具体处理步骤没有在这一段代码中展示）。
    循环中还会更新进度条my_progress_bar。
  */
  for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
    iter_image != vec_image.end();
    ++iter_image, ++my_progress_bar )
  {
    // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
    width = height = ppx = ppy = focal = -1.0;
    // 获取含路径的文件名sImageFilename和不含路径的文件名sImFilenamePart
    const std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );
    const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

    // 用GetFormat来Test if the image format is supported:
    if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
    {
      error_report_stream
          << sImFilenamePart << ": Unkown image file format." << "\n";
      continue; // image cannot be opened所以就continue到下一个图片
    }
    //判断当前文件是否为掩膜图像。掩膜图像通常用于指示在图像处理或分析时应该忽略的区域。如果发现文件名包含这些标识，同样记录到error_report_stream中，并输出提示信息，然后跳过此图像的进一步处理。
    if (sImFilenamePart.find("mask.png") != std::string::npos
       || sImFilenamePart.find("_mask.png") != std::string::npos)
    {
      error_report_stream
          << sImFilenamePart << " is a mask image" << "\n";
      continue;
    }
    //读取头信息，其中包括图像width和height。ppx和ppy是主点（中心点）的坐标
    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
      continue; // image cannot be read
    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;


    // 尝试从sKmatrix中获取焦距focal，主点ppx和ppy。如果校准信息无效，则将focal设为-1.0。你知道，sKmatrix都是空串。转到focal == -1
    if (sKmatrix.size() > 0) // Known user calibration K matrix
    {
      if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
        focal = -1.0;
    }
    else // User provided focal length value
      if (focal_pixels != -1 )
        focal = focal_pixels;

    // If not manually provided or wrongly provided，正常啦，这时要尝试从图像的EXIF元数据中读取焦距信息
    if (focal == -1)
    {
      std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
      exifReader->open( sImageFilename );
      //用exifReader（智能（？）指针所以用箭头）的成员函数检查有没有合适的exif：有品牌且有型号
      const bool bHaveValidExifMetadata =
        exifReader->doesHaveExifInfo()
        && !exifReader->getModel().empty()
        && !exifReader->getBrand().empty();
      /*
         如果EXIF数据中的焦距值为0，记录错误信息，并将focal设为-1.0。
         如果EXIF中有有效焦距信息，则通过查询前面加载的vec_database数据库，获取相应相机模型的传感器尺寸（sensorSize_），以计算实际的焦距值。这通过图像最长边与传感器尺寸的比例乘以EXIF中的焦距值来完成。
         如果数据库中不存在对应的相机模型，记录错误信息，建议用户将相机模型和传感器宽度添加到数据库中。
       */
      if (bHaveValidExifMetadata) // If image contains meta data
      {
        // Handle case where focal length is equal to 0
        if (exifReader->getFocal() == 0.0f)
        {
          error_report_stream
            << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
          focal = -1.0;
        }
        else
        // Create the image entry in the list file
        {
            //品牌模型字符串
          const std::string sCamModel = exifReader->getBrand() + " " + exifReader->getModel();

          Datasheet datasheet;
          if ( getInfo( sCamModel, vec_database, datasheet ))
          {
            // The camera model was found in the database so we can compute it's approximated focal length
            //focal = max(w, h) * F / S,其中F为相机焦距长度(focal length，单位毫米mm)，w, h为图片的宽高，S为ccd传感器孔径（sensor size，单位mm）
            //算出来*以像素为单位的*焦距
            const double ccdw = datasheet.sensorSize_;
            focal = std::max ( width, height ) * exifReader->getFocal() / ccdw;
          }
          //如果你用小米相机又妹有加数据的话就报错
          else
          {
            error_report_stream
              << stlplus::basename_part(sImageFilename)
              << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
              << "Please consider add your camera model and sensor width in the database." << "\n";
          }
        }
      }
    }
    // Build intrinsic parameter related to the view
    // IntrinsicBase类的指针intrinsic将会储存EXIF里的内参信息
    std::shared_ptr<IntrinsicBase> intrinsic;

    if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
    {
      // Create the desired camera type
        //std::make_shared是C++11引入的一种智能指针，它用于创建一个指向动态分配的对象的std::shared_ptr智能指针。
        //坏了，这用python能翻译吗
      switch (e_User_camera_model)
      {
        case PINHOLE_CAMERA:
          intrinsic = std::make_shared<Pinhole_Intrinsic>
            (width, height, focal, ppx, ppy);
        break;
        case PINHOLE_CAMERA_RADIAL1:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_RADIAL3:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_BROWN:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_FISHEYE:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
        break;
        case CAMERA_SPHERICAL:
           intrinsic = std::make_shared<Intrinsic_Spherical>
             (width, height);
        break;
        default:
          OPENMVG_LOG_ERROR << "Error: unknown camera model: " << (int) e_User_camera_model;
          return EXIT_FAILURE;
      }
    }

    // 构建与图像相对应的视图，并且根据图像是否具有有效的GPS位置信息来处理视图。
    Vec3 pose_center;
    //getGPS返回false所以这个if不用看了，跳else吧
    if (getGPS(sImageFilename, i_GPS_XYZ_method, pose_center) && b_Use_pose_prior)
    {
      ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

      // Add intrinsic related to the image (if any)
      if (!intrinsic)
      {
        //Since the view have invalid intrinsic data
        // (export the view, with an invalid intrinsic field value)
        v.id_intrinsic = UndefinedIndexT;
      }
      else
      {
        // Add the defined intrinsic to the sfm_container
        intrinsics[v.id_intrinsic] = intrinsic;
      }

      v.b_use_pose_center_ = true;
      v.pose_center_ = pose_center;
      // prior weights
      if (prior_w_info.first == true)
      {
        v.center_weight_ = prior_w_info.second;
      }

      // Add the view to the sfm_container
      views[v.id_view] = std::make_shared<ViewPriors>(v);
    }
    // 如果没有有效的GPS信息，或不使用位置先验，代码会创建一个普通的View对象。其中会包含当前图像的内参信息intrinsic指针
    // 而视图v创建的智能指针将与intrinsic指针分别存放在sfm_data.views和sfm_data.intrinsics中。
    // 通过使用std::shared_ptr智能指针，代码自动管理视图对象的生命周期，确保资源被正确释放，避免内存泄漏。问题是，到底什么时候释放，要是太早释放那sfm_data.json存的指针就没用了。
    // iter_image和这些参数作用域可真大
    else
    {
      View v(*iter_image, views.size(), views.size(), views.size(), width, height);

      // Add intrinsic related to the image (if any)
      if (!intrinsic)//不至于
      {
        //Since the view have invalid intrinsic data
        // (export the view, with an invalid intrinsic field value)
        v.id_intrinsic = UndefinedIndexT;
      }
      else
      {
        // Add the defined intrinsic to the sfm_container，
        // 其中intrinsics是sfm_data.intrinsics成员的引用，存放v视图中内参id以及对应的内参指针构成的键值对。
        intrinsics[v.id_intrinsic] = intrinsic;
      }

      // Add the view to the sfm_container，
      // 其中views是sfm_data.views成员的引用，存放v视图中视图id以及对应的视图指针构成的键值对。
      views[v.id_view] = std::make_shared<View>(v);
    }
  }

  // Display saved warning & error messages if any.咱不需要吧。
  if (!error_report_stream.str().empty())
  {
    OPENMVG_LOG_WARNING
      << "Warning & Error messages:\n"
      << error_report_stream.str();
  }

  // Group camera that share common properties if desired (leads to more faster & stable BA).那当然了，都是一个相机拍的。
  if (b_Group_camera_model)
  {
    GroupSharedIntrinsics(sfm_data);
  }

  // Store SfM_Data views & intrinsic data，用Save函数
  if (!Save(
    sfm_data, //所有图片的视图和内参信息
    stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),  //确认输出文件路径并创建sfm_data.json
    ESfM_Data(VIEWS|INTRINSICS)))  // 通过VIEWS|INTRINSICS标志，指示函数仅保存视图和内参信息。
  {
    return EXIT_FAILURE;
  }

  OPENMVG_LOG_INFO //打小报告
    << "SfMInit_ImageListing report:\n"
    << "listed #File(s): " << vec_image.size() << "\n"
    << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
    << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size();

  return EXIT_SUCCESS;
}
