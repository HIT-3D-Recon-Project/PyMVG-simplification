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
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <pybind11/stl.h>
//标准库头文件

namespace py = pybind11;
using namespace std;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;
//使用命名空间

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

int main(int argc, char **argv)
{
  py::scoped_interpreter guard{};

  auto python_code = py::module_::import("main_SfMInit_ImageListing");  // 加载Python模块
  auto read_datafile = python_code.attr("read_datafile");  // 获取Python函数
  auto read_input = python_code.attr("read_input");
  auto read_output = python_code.attr("read_output");
  auto search_size = python_code.attr("search_size");
  auto files_name = python_code.attr("files_name");
  auto help_log = python_code.attr("help_log");

  std::string sImageDir, sfileDatabase = "", sOutputDir = "";

  std::string sPriorWeights = "1.0;1.0;1.0";
  std::pair<bool, Vec3> prior_w_info(false, Vec3());
  
  bool b_Group_camera_model = true;

  int i_GPS_XYZ_method = 0;

  double focal_pixels = -1.0;
  double sensorSize = 0.0;

  auto ImageDir = read_input();
  auto fileDatabase = read_datafile();
  auto OutputDir = read_output();

  sImageDir = py::str(ImageDir).cast<std::string>();
  sfileDatabase = py::str(fileDatabase).cast<std::string>();
  sOutputDir = py::str(OutputDir).cast<std::string>();

  if (sImageDir == "ERROR" || sfileDatabase == "ERROR" || sOutputDir == "ERROR") {
      // 处理错误情况
      return EXIT_FAILURE;
  }
  
  //此处应有异常处理，怀疑不会输出……
  OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-d|--sensorWidthDatabase]\n"
      << "[-o|--outputDirectory]\n";

  double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;
  
  const EINTRINSIC e_User_camera_model = EINTRINSIC(PINHOLE_CAMERA_RADIAL3);

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
  
  //存图像名字到向量vec_image并排序
  py::object result = files_name(sImageDir); // 调用Python函数并获取结果
  std::vector<std::string> vec_image = py::cast<std::vector<std::string>>(result); // 转换结果

  SfM_Data sfm_data;
  sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views; 
  Intrinsics & intrinsics = sfm_data.intrinsics;

  //实例化一个进度条my_progress_bar，用于跟踪和显示列出图像的进度。
  //创建一个std::ostringstream实例error_report_stream，用于收集过程中可能发生的错误信息。
  system::LoggerProgress my_progress_bar(vec_image.size(), "- Listing images -" );
  std::ostringstream error_report_stream;

  for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
    iter_image != vec_image.end();
    ++iter_image, ++my_progress_bar )
  {
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
    
    //读取头信息，其中包括图像width和height。ppx和ppy是主点（中心点）的坐标
    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
      continue; // image cannot be read
    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;


    //此处focal要改一改
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

          if (py::object size = search_size(sCamModel, sfileDatabase))
          {
            //focal = max(w, h) * F / S,其中F为相机焦距长度(focal length，单位毫米mm)，w, h为图片的宽高，S为ccd传感器孔径（sensor size，单位mm）
            //算出来*以像素为单位的*焦距
            sensorSize = py::cast<double>(size);
            const double ccdw = sensorSize;
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
      intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
        (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);
      
    // 构建与图像相对应的视图，并且根据图像是否具有有效的GPS位置信息来处理视图。
    Vec3 pose_center;

    View v(*iter_image, views.size(), views.size(), views.size(), width, height);

    intrinsics[v.id_intrinsic] = intrinsic;
    views[v.id_view] = std::make_shared<View>(v);
  }

  if (!error_report_stream.str().empty())
  {
    OPENMVG_LOG_WARNING
      << "Warning & Error messages:\n"
      << error_report_stream.str();
  }
  GroupSharedIntrinsics(sfm_data);

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
