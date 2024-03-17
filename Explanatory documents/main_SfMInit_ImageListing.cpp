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
//openMVG��ͷ�ļ�
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
//��������ͷ�ļ�
#include <fstream>
#include <memory>
#include <string>
#include <utility>
//��׼��ͷ�ļ�
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;
//ʹ�������ռ�
/// ���������ڲξ���K matrix���ǲ��� "f;0;ppx;0;f;ppy;0;0;1" ��ʽ���ַ���
/// �Լ����е� f,ppx,ppy �Ƿ�����Ч��ֵ
//Ȼ��������pipeline��û������sKmatrix���Ǻ��񲻻ᴥ�������
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
    ��������Ϊbool getGPS����������������filename��һ���ַ�����ָ��ͼƬ�ļ���·������GPS_to_XYZ_method��һ��������ָ����GPS����ת������ά�ռ��ķ��������Լ�pose_center��һ��Vec3���͵����ã����ڴ洢ת�������ά�ռ�㣩��

    ���ȣ�����һ��Exif_IO���͵�����ָ��exifReader����ʹ��Exif_IO_EasyExif���캯����ʼ������һ����Ϊ�˶�ȡͼƬ�ļ��е�EXIF��Ϣ��EXIF��Ϣͨ������������ʱ�ĸ������ú�����������GPS���ꡣ

    ��������ͨ��if (exifReader)�ж�exifReader�Ƿ�ɹ�������

    ��if����ڲ������Դ�ָ����ͼƬ�ļ��������ļ��Ƿ����EXIF��Ϣ��ͨ������exifReader->open(filename)��exifReader->doesHaveExifInfo()ʵ�֡�

    ����ļ��ɹ��򿪲��Ұ���EXIF��Ϣ����ô��һ������Ƿ����GPS������Ϣ������ͨ������exifReader->GPSLatitude(&latitude), exifReader->GPSLongitude(&longitude)��exifReader->GPSAltitude(&altitude)ʵ�ֵģ��ֱ��ȡγ�ȡ����Ⱥ͸߶���Ϣ��

    ����ɹ���ȡ��GPS���꣬����GPS_to_XYZ_method��ֵ��ѡ��ͬ��ת��������GPS����ת��Ϊ��ά�ռ��еĵ㡣case 1:�ǽ�GPS����ת��ΪUTM��ͨ�ú���ī���У����꣬case 0:��default:�ǽ�GPS����ת��ΪECEF���������ĵع�����ϵ�����ꡣת��������걻��ֵ��pose_center��

    ���һ��˳������GPS���걻�ɹ�ת������������true��ʾ�ɹ���ȡ��ת��GPS����;������κβ����г���ʧ�ܣ����޷����ļ����ļ��в�����EXIF��Ϣ�򲻰���GPS��Ϣ�����������շ���false����ʾû�гɹ���ȡGPS���ꡣ
*/
//��������ܲ��ܳ������������ά����pose_center�Ĺ�����̡��ҿ����£��ֻ��ĵ���Ƭ��EXIF��Ϣ������û�о�γ�ȡ���Ŀ���ĳǱ�ʵ��Ҳû�У��������getGPS�������Ǹ�false��������
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
// ��ͷϷ����������ͼ��sImageDir��ģ���������ݿ⣨sensor_width_camera_database.txt�������sfm_data.json����ͼ��ߴ磬���ࣨ��λ���أ�
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sfileDatabase = "",
    sOutputDir = "",
    sKmatrix;

  std::string sPriorWeights = "1.0;1.0;1.0";
  std::pair<bool, Vec3> prior_w_info(false, Vec3());

  //ö�ٳ���PINHOLE_CAMERA_RADIAL3Ĭ��ֵΪ3����pipeline�Ĳ�������Ҳ��������ˡ����Ծ���3���ѡ�
  //�������溣�ݺ�GitHub�ϵķ�����3ָ���Ǻ��С��������K1,K2,K3�������ģ�ͣ�������ģ������������У���������ͼ��ʱ�����Ļ��䣬�ر����ڱ�Ե���򡣡�
  //�Լ�����һЩĬ��ֵ��
  //b_Group_camera_model��һ������ֵ������ָʾ�Ƿ�Ӧ�ö�ʹ����ͬ���ģ�͵�������з��顣�������Ϊtrue������ζ��ϵͳ�ڴ�����ͼ��ʱ���Ὣʹ����ͬģ�͵������Ϊһ�飬���������Ż��������̺����Ч�ʡ�
  //i_GPS_XYZ_methodָ���ǽ�GPS���꣨��γ�Ⱥ͸߶ȣ�ת��Ϊ��ά�ռ����꣨XYZ���ķ����������0���ܱ�ʾʹ�����ض���ת������������������ĵع�����ϵ��ECEF������������ͬ��ת���������������ڲ�ͬ��Ӧ�ó����;���Ҫ��
  //focal_pixels����������������ֵ�������������ͷ���Ե�һ���ؼ�����������ͼ�������ά�ؽ��ǳ���Ҫ�������-1.0���ܱ�ʾ��ֵδ����ʼ������Ҫͨ��������ʽȷ������ʵ��Ӧ���У�������Դ�EXIF�����ж�ȡ������ͨ�������������ơ�
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
  
  bool b_Group_camera_model = true;//1-> (default) view can share some camera intrinsic parameters

  int i_GPS_XYZ_method = 0;

  double focal_pixels = -1.0;
  //Ȼ����ֻ����ǰ����������������ò��û��ʲôsKmatrix��
  //�������Ϊd�Ĳ����������˼�¼�����ϸ��ֳ���������������ߴ硱��sensor_width_camera_database.txt�����ֵ�һ�����Թ���ѯ���Ǳ�ͼƬ��������ͺ�Ҳ��¼�ˣ�ֵ��5.773�����ǣ����ֻ�������ͺ��������Ҳ�����
  //Ҳ����˵��Ҫô�������Ҳ�п��ܻ����Ҳ�������Ҫô��취�����ĳ̨�ֻ��ġ��������ߴ硱��
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
  //���û�乻�������ͱ����ҵ���help������ô�á�������
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
  //'P', "use_pose_prior"����˵������λ�˾�b_Use_pose_prior��Ϊ1
  const bool b_Use_pose_prior = cmd.used('P');
  //������Ϣ��ȷ��
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

  // ÿ��ͼƬ�ľ������
  double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;
  // ȷ���������ģ�ͣ���֪���ڸ�ʲô
  const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);
  // �����������·���Ų�
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
  //���sKmatrix�ϲ��Ϲ�񣬵���Ĭ��ֵ�ǿյģ����Բ���顣
  if (sKmatrix.size() > 0 &&
    !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy) )
  {
    OPENMVG_LOG_ERROR << "Invalid K matrix input";
    return EXIT_FAILURE;
  }
  //��k��f������һ���
  if (sKmatrix.size() > 0 && focal_pixels != -1.0)
  {
    OPENMVG_LOG_ERROR << "Cannot combine -f and -k options";
    return EXIT_FAILURE;
  }
  //�����Ϊ�գ�����ʹ��parseDatabase��������������ݿ��ļ��������н���Ч����ת��ΪDatasheet������䵽����vec_database�С���������ת�ơ�
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

  // ����û�˵b_Use_pose_prior�������checkPriorWeightsString���������ַ���sPriorWeights������ַ���Ӧ�ð�������λ�˹��Ƶ�����Ȩ�أ�Ĭ��ֵ111�������������pair���洢��prior_w_info�С�
  if (b_Use_pose_prior)
  {
    prior_w_info = checkPriorWeightsString(sPriorWeights);
  }
  //��ͼ�����ֵ�����vec_image������
  std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
  std::sort(vec_image.begin(), vec_image.end());

  // SfM_Dataʵ������������������ֻ��ͼ���Ŀ¼��Ϊ���Դ���
  // ��������ͼ���ڲε����ã������ҵ����ڻ���֪�����ù��ܵľ�������
  SfM_Data sfm_data;
  sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views; 
  Intrinsics & intrinsics = sfm_data.intrinsics;

  //ʵ����һ��������my_progress_bar�����ڸ��ٺ���ʾ�г�ͼ��Ľ��ȡ�
  //����һ��std::ostringstreamʵ��error_report_stream�������ռ������п��ܷ����Ĵ�����Ϣ��
  system::LoggerProgress my_progress_bar(vec_image.size(), "- Listing images -" );
  std::ostringstream error_report_stream;

  /*
    ʹ��һ��ѭ������vec_image�����е�ÿһ��ͼ���ļ�����
    ��ѭ���ڲ������ÿ��ͼ���ļ����д������崦����û������һ�δ�����չʾ����
    ѭ���л�����½�����my_progress_bar��
  */
  for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
    iter_image != vec_image.end();
    ++iter_image, ++my_progress_bar )
  {
    // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
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
    //�жϵ�ǰ�ļ��Ƿ�Ϊ��Ĥͼ����Ĥͼ��ͨ������ָʾ��ͼ��������ʱӦ�ú��Ե�������������ļ���������Щ��ʶ��ͬ����¼��error_report_stream�У��������ʾ��Ϣ��Ȼ��������ͼ��Ľ�һ������
    if (sImFilenamePart.find("mask.png") != std::string::npos
       || sImFilenamePart.find("_mask.png") != std::string::npos)
    {
      error_report_stream
          << sImFilenamePart << " is a mask image" << "\n";
      continue;
    }
    //��ȡͷ��Ϣ�����а���ͼ��width��height��ppx��ppy�����㣨���ĵ㣩������
    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
      continue; // image cannot be read
    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;


    // ���Դ�sKmatrix�л�ȡ����focal������ppx��ppy�����У׼��Ϣ��Ч����focal��Ϊ-1.0����֪����sKmatrix���ǿմ���ת��focal == -1
    if (sKmatrix.size() > 0) // Known user calibration K matrix
    {
      if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
        focal = -1.0;
    }
    else // User provided focal length value
      if (focal_pixels != -1 )
        focal = focal_pixels;

    // If not manually provided or wrongly provided������������ʱҪ���Դ�ͼ���EXIFԪ�����ж�ȡ������Ϣ
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

          Datasheet datasheet;
          if ( getInfo( sCamModel, vec_database, datasheet ))
          {
            // The camera model was found in the database so we can compute it's approximated focal length
            //focal = max(w, h) * F / S,����FΪ������೤��(focal length����λ����mm)��w, hΪͼƬ�Ŀ�ߣ�SΪccd�������׾���sensor size����λmm��
            //�����*������Ϊ��λ��*����
            const double ccdw = datasheet.sensorSize_;
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

    // ������ͼ�����Ӧ����ͼ�����Ҹ���ͼ���Ƿ������Ч��GPSλ����Ϣ��������ͼ��
    Vec3 pose_center;
    //getGPS����false�������if���ÿ��ˣ���else��
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
    // ���û����Ч��GPS��Ϣ����ʹ��λ�����飬����ᴴ��һ����ͨ��View�������л������ǰͼ����ڲ���Ϣintrinsicָ��
    // ����ͼv����������ָ�뽫��intrinsicָ��ֱ�����sfm_data.views��sfm_data.intrinsics�С�
    // ͨ��ʹ��std::shared_ptr����ָ�룬�����Զ�������ͼ������������ڣ�ȷ����Դ����ȷ�ͷţ������ڴ�й©�������ǣ�����ʲôʱ���ͷţ�Ҫ��̫���ͷ���sfm_data.json���ָ���û���ˡ�
    // iter_image����Щ��������������
    else
    {
      View v(*iter_image, views.size(), views.size(), views.size(), width, height);

      // Add intrinsic related to the image (if any)
      if (!intrinsic)//������
      {
        //Since the view have invalid intrinsic data
        // (export the view, with an invalid intrinsic field value)
        v.id_intrinsic = UndefinedIndexT;
      }
      else
      {
        // Add the defined intrinsic to the sfm_container��
        // ����intrinsics��sfm_data.intrinsics��Ա�����ã����v��ͼ���ڲ�id�Լ���Ӧ���ڲ�ָ�빹�ɵļ�ֵ�ԡ�
        intrinsics[v.id_intrinsic] = intrinsic;
      }

      // Add the view to the sfm_container��
      // ����views��sfm_data.views��Ա�����ã����v��ͼ����ͼid�Լ���Ӧ����ͼָ�빹�ɵļ�ֵ�ԡ�
      views[v.id_view] = std::make_shared<View>(v);
    }
  }

  // Display saved warning & error messages if any.�۲���Ҫ�ɡ�
  if (!error_report_stream.str().empty())
  {
    OPENMVG_LOG_WARNING
      << "Warning & Error messages:\n"
      << error_report_stream.str();
  }

  // Group camera that share common properties if desired (leads to more faster & stable BA).�ǵ�Ȼ�ˣ�����һ������ĵġ�
  if (b_Group_camera_model)
  {
    GroupSharedIntrinsics(sfm_data);
  }

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
