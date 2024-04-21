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
//openMVG��ͷ�ļ�
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
//��������ͷ�ļ�
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <pybind11/stl.h>
//��׼��ͷ�ļ�

namespace py = pybind11;
using namespace std;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;
//ʹ�������ռ�

/// ���Խ�Ԥ��Ȩ���ַ���sWeights��ɢ��vec_strȻ�����뵽����ֵ������-�����ԣ��ﷵ�ء���Ȼ����û���룬����main�����˳�ʼֵ�����Խ����<True,����1.0��ɵ�����>��
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

  auto python_code = py::module_::import("main_SfMInit_ImageListing");  // ����Pythonģ��
  auto read_datafile = python_code.attr("read_datafile");  // ��ȡPython����
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
      // ����������
      return EXIT_FAILURE;
  }
  
  //�˴�Ӧ���쳣�������ɲ����������
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
  
  //��ͼ�����ֵ�����vec_image������
  py::object result = files_name(sImageDir); // ����Python��������ȡ���
  std::vector<std::string> vec_image = py::cast<std::vector<std::string>>(result); // ת�����

  SfM_Data sfm_data;
  sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views; 
  Intrinsics & intrinsics = sfm_data.intrinsics;

  //ʵ����һ��������my_progress_bar�����ڸ��ٺ���ʾ�г�ͼ��Ľ��ȡ�
  //����һ��std::ostringstreamʵ��error_report_stream�������ռ������п��ܷ����Ĵ�����Ϣ��
  system::LoggerProgress my_progress_bar(vec_image.size(), "- Listing images -" );
  std::ostringstream error_report_stream;

  for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
    iter_image != vec_image.end();
    ++iter_image, ++my_progress_bar )
  {
    width = height = ppx = ppy = focal = -1.0;
    // ��ȡ��·�����ļ���sImageFilename�Ͳ���·�����ļ���sImFilenamePart
    const std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );
    const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

    // ��GetFormat��Test if the image format is supported:
    if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
    {
      error_report_stream
          << sImFilenamePart << ": Unkown image file format." << "\n";
      continue; // image cannot be opened���Ծ�continue����һ��ͼƬ
    }
    
    //��ȡͷ��Ϣ�����а���ͼ��width��height��ppx��ppy�����㣨���ĵ㣩������
    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
      continue; // image cannot be read
    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;


    //�˴�focalҪ��һ��
    if (focal == -1)
    {
      std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
      exifReader->open( sImageFilename );
      //��exifReader�����ܣ�����ָ�������ü�ͷ���ĳ�Ա���������û�к��ʵ�exif����Ʒ�������ͺ�
      const bool bHaveValidExifMetadata =
        exifReader->doesHaveExifInfo()
        && !exifReader->getModel().empty()
        && !exifReader->getBrand().empty();
      /*
         ���EXIF�����еĽ���ֵΪ0����¼������Ϣ������focal��Ϊ-1.0��
         ���EXIF������Ч������Ϣ����ͨ����ѯǰ����ص�vec_database���ݿ⣬��ȡ��Ӧ���ģ�͵Ĵ������ߴ磨sensorSize_�����Լ���ʵ�ʵĽ���ֵ����ͨ��ͼ������봫�����ߴ�ı�������EXIF�еĽ���ֵ����ɡ�
         ������ݿ��в����ڶ�Ӧ�����ģ�ͣ���¼������Ϣ�������û������ģ�ͺʹ����������ӵ����ݿ��С�
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
            //Ʒ��ģ���ַ���
          const std::string sCamModel = exifReader->getBrand() + " " + exifReader->getModel();

          if (py::object size = search_size(sCamModel, sfileDatabase))
          {
            //focal = max(w, h) * F / S,����FΪ������೤��(focal length����λ����mm)��w, hΪͼƬ�Ŀ�ߣ�SΪccd�������׾���sensor size����λmm��
            //�����*������Ϊ��λ��*����
            sensorSize = py::cast<double>(size);
            const double ccdw = sensorSize;
            focal = std::max ( width, height ) * exifReader->getFocal() / ccdw;
          }
          //�������С����������м����ݵĻ��ͱ���
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
    // IntrinsicBase���ָ��intrinsic���ᴢ��EXIF����ڲ���Ϣ
    std::shared_ptr<IntrinsicBase> intrinsic;

    if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
    {
      // Create the desired camera type
        //std::make_shared��C++11�����һ������ָ�룬�����ڴ���һ��ָ��̬����Ķ����std::shared_ptr����ָ�롣
        //���ˣ�����python�ܷ�����
      intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
        (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);
      
    // ������ͼ�����Ӧ����ͼ�����Ҹ���ͼ���Ƿ������Ч��GPSλ����Ϣ��������ͼ��
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

  // Store SfM_Data views & intrinsic data����Save����
  if (!Save(
    sfm_data, //����ͼƬ����ͼ���ڲ���Ϣ
    stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),  //ȷ������ļ�·��������sfm_data.json
    ESfM_Data(VIEWS|INTRINSICS)))  // ͨ��VIEWS|INTRINSICS��־��ָʾ������������ͼ���ڲ���Ϣ��
  {
    return EXIT_FAILURE;
  }

  OPENMVG_LOG_INFO //��С����
    << "SfMInit_ImageListing report:\n"
    << "listed #File(s): " << vec_image.size() << "\n"
    << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
    << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size();

  return EXIT_SUCCESS;
}
