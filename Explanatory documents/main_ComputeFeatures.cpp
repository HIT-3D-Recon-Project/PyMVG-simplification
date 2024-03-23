// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// The <cereal/archives> headers are special and must be included first.
/*
 功能：
 两幅图像查找特征点并且特征点的匹配与绘制
 本文给出了三种描述子SIFT、AKAZE、AKAZE_MLDB

 输入：
 两幅图像，描述子种类
 
 输入参数：
 -i/--input_file: 输入sfm描述文件 sfm_data.json
 -o/--outdir: 输出目录：比如特征描述文件

 -m/--describerMethod：图像特征描述方法； 
    SIFT (默认下) 
    SIFT_ANATOMY
    AKAZE_FLOAT: AKAZE浮点数描述
    AKAZE_MLDB:  AKAZE二进制描述
 -u/--upright：AKAZE描述子使用，是否计算方向
 -f/--fore：是否强制重新计算特征点和描述子
 -p/--describerPreset：描述子质量：NORMAL，HIGH，ULTRA
 -n/--numThreads：执行的thread个数(使用openMP才需要设置)

 输出：
 IndMatches：vector，存储的类型为IndMatch
 IndMatch的数据为uint32_t的x和y

 附加：
 执行匹配使用的是查找最近邻，用距离比过滤
 */


#include <cereal/archives/json.hpp>//c++序列化库

#include "openMVG/features/akaze/image_describer_akaze_io.hpp"//引入AKAZE特征描述器

#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"//引入SIFT特征描述器
#include "openMVG/image/image_io.hpp"//引入图像输入输出功能
#include "openMVG/features/regions_factory_io.hpp"//引用Regions类，用于存储特征点和描述子
#include "openMVG/sfm/sfm_data.hpp"//SFM数据
#include "openMVG/sfm/sfm_data_io.hpp"//SFM数据IO
#include "openMVG/system/logger.hpp"//日志记录
#include "openMVG/system/loggerprogress.hpp"//进度日志记录
#include "openMVG/system/timer.hpp"//系统计时器

#include "third_party/cmdLine/cmdLine.h"//第三方命令行解析
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"//第三方简化文件系统

#include "nonFree/sift/SIFT_describer_io.hpp"//SIFT特征描述器IO

#include <cereal/details/helpers.hpp>//cereal辅助库

#include <atomic>//c++原子操作类型
#include <cstdlib>
#include <fstream>
#include <string>

#ifdef OPENMVG_USE_OPENMP//检查是否定义了名为 OPENMVG_USE_OPENMP 的宏
#include <omp.h>//openMP库
#endif

using namespace openMVG;//使用openMVG命名空间
using namespace openMVG::image;//使用图像命名空间
using namespace openMVG::features;//使用特征命名空间
using namespace openMVG::sfm;//使用SFM命名空间

//指定图像特征描述器的预设质量级别
features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)//将传入的字符串参数 sPreset 转换为 features::EDESCRIBER_PRESET 类型的枚举值
{
  features::EDESCRIBER_PRESET preset;
  if (sPreset == "NORMAL")
    preset = features::NORMAL_PRESET;//使用正常质量级别的特征提取预设
  else
  if (sPreset == "HIGH")
    preset = features::HIGH_PRESET;//使用高质量级别的特征提取预设
  else
  if (sPreset == "ULTRA")
    preset = features::ULTRA_PRESET;//使用最高质量级别的特征提取预设
  else
    preset = features::EDESCRIBER_PRESET(-1);//处理未知或者不支持的预设值输入
  return preset;
}

/// - 计算视图图像描述（特征和描述符提取）
/// - 导出计算数据
int main(int argc, char **argv)
{
  CmdLine cmd;//定义命令行参数

  std::string sSfM_Data_Filename;//输入文件名
  std::string sOutDir = "";//输出目录
  bool bUpRight = false;//是否使用UpRight特征
  std::string sImage_Describer_Method = "SIFT";//特征描述方法，默认为SIFT
  bool bForce = false;// 是否强制重新计算特征
  std::string sFeaturePreset = "";// 特征预设

#ifdef OPENMVG_USE_OPENMP
  int iNumThreads = 0;// 线程数，用于OpenMP并行计算
#endif
  
  // 添加命令行选项
  
  // 必需的选项
  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('o', sOutDir, "outdir") );//用户必须指定输入文件（sSfM_Data_Filename）和输出目录（sOutDir）
  
  // 可选的选项
  cmd.add( make_option('m', sImage_Describer_Method, "describerMethod") );//'m' 特征描述方法（sImage_Describer_Method），默认为SIFT
  cmd.add( make_option('u', bUpRight, "upright") );//'u' 是否使用 upright 特征（bUpRight），默认为 false
  cmd.add( make_option('f', bForce, "force") );//'f' 是否强制重新计算特征（bForce），默认为 false
  cmd.add( make_option('p', sFeaturePreset, "describerPreset") );//'p' 特征预设（sFeaturePreset），用于指定特征提取的详细程度

#ifdef OPENMVG_USE_OPENMP
  cmd.add( make_option('n', iNumThreads, "numThreads") );
#endif

  try {
      // 处理命令行参数
      
      if (argc == 1) throw std::string("Invalid command line parameter.");//检查命令行参数的数量。如果只有一个参数，则抛出一个异常
      cmd.process(argc, argv);//如果命令行参数的数量大于1，则调用cmd.process函数处理这些参数
  } 
  catch (const std::string& s) {
      
      //捕获try模块抛出的类型为std::string的异常，并将其值存储在变量s中
      // 使用 OPENMVG_LOG_INFO 和 OPENMVG_LOG_ERROR 宏来记录信息和错误消息
      
      OPENMVG_LOG_INFO
        << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] a SfM_Data file \n"
        << "[-o|--outdir path] \n"
        << "\n[Optional]\n"
        << "[-f|--force] Force to recompute data\n"
        << "[-m|--describerMethod]\n"
        << "  (method to use to describe an image):\n"
        << "   SIFT (default),\n"
        << "   SIFT_ANATOMY,\n"
        << "   AKAZE_FLOAT: AKAZE with floating point descriptors,\n"
        << "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
        << "[-u|--upright] Use Upright feature 0 or 1\n"
        << "[-p|--describerPreset]\n"
        << "  (used to control the Image_describer configuration):\n"
        << "   NORMAL (default),\n"
        << "   HIGH,\n"
        << "   ULTRA: !!Can take long time!!\n"
#ifdef OPENMVG_USE_OPENMP
        << "[-n|--numThreads] number of parallel computations\n"
#endif
      ;

      OPENMVG_LOG_ERROR << s;//输出错误信息
      return EXIT_FAILURE;//返回一个失败状态码（1）
  }

  OPENMVG_LOG_INFO
      // 日志输出调用信息
    << " You called : " << "\n"
    << argv[0] << "\n" 
    << "--input_file " << sSfM_Data_Filename << "\n" 
    << "--outdir " << sOutDir << "\n"
    << "--describerMethod " << sImage_Describer_Method << "\n"
    << "--upright " << bUpRight << "\n"
    << "--describerPreset " << (sFeaturePreset.empty() ? "NORMAL" : sFeaturePreset) << "\n" //用户选择的描述子预设值，默认为NORMAL
    << "--force " << bForce << "\n"
#ifdef OPENMVG_USE_OPENMP
    << "--numThreads " << iNumThreads << "\n" //使用OpenMP进行并行计算时的线程数量
#endif
    ;

  // 检查输出目录
  if (sOutDir.empty())//输出目录是否为空
  {
    OPENMVG_LOG_ERROR << "\nIt is an invalid output directory";
    return EXIT_FAILURE;
  }

  //创建输出目录
  if (!stlplus::folder_exists(sOutDir))//输出目录是否存在
  {
    if (!stlplus::folder_create(sOutDir))//若不存在，创建该目录
    {
      OPENMVG_LOG_ERROR << "Cannot create output directory";//创建失败，输出错误日志
      return EXIT_FAILURE;
    }
  }

  //---------------------------------------
  // a.加载输入场景
  //---------------------------------------
  SfM_Data sfm_data;
  //从指定的文件路径sSfM_Data_Filename加载一个SfM数据对象sfm_data
  
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    OPENMVG_LOG_ERROR
      << "The input file \""<< sSfM_Data_Filename << "\" cannot be read";
    return EXIT_FAILURE;
  }

  // b. 初始化图像描述器
  // - 在预先计算特征的情况下检索使用过的特征
  // - 否则创建所需的

  using namespace openMVG::features;
  std::unique_ptr<Image_describer> image_describer;

  
  const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
  if (!bForce && stlplus::is_file(sImage_describer))
  {
    // 从文件中动态加载图像描述器（将恢复旧的使用设置）
    std::ifstream stream(sImage_describer.c_str());
    if (!stream)
      return EXIT_FAILURE;

    try
    {
      cereal::JSONInputArchive archive(stream);
      archive(cereal::make_nvp("image_describer", image_describer));
    }
    catch (const cereal::Exception & e)
    {
      OPENMVG_LOG_ERROR << e.what() << '\n'
        << "Cannot dynamically allocate the Image_describer interface.";
      return EXIT_FAILURE;
    }
  }
  else
  {
    // 如果不存在预先计算的描述器或指定了强制重新计算，将根据用户指定方法创建图像描述器
    // 不使用工厂，直接分配（考虑了是否使用Upright特征）
    if (sImage_Describer_Method == "SIFT")
    {
      image_describer.reset(new SIFT_Image_describer
        (SIFT_Image_describer::Params(), !bUpRight));
    }
    else
    if (sImage_Describer_Method == "SIFT_ANATOMY")
    {
      image_describer.reset(
        new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));
    }
    else
    if (sImage_Describer_Method == "AKAZE_FLOAT")
    {
      image_describer = AKAZE_Image_describer::create
        (AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF), !bUpRight);
    }
    else
    if (sImage_Describer_Method == "AKAZE_MLDB")
    {
      image_describer = AKAZE_Image_describer::create
        (AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MLDB), !bUpRight);
    }
    
    //检查图像描述器是否创建成功
    if (!image_describer)
    {
      OPENMVG_LOG_ERROR << "Cannot create the designed Image_describer:"
        << sImage_Describer_Method << ".";
      return EXIT_FAILURE;
    }
    else
    {
      // 设置描述器的配置
      if (!sFeaturePreset.empty())//检查是否为用户提供了特征预设值
      //尝试用该预设值配置图像描述器
      if (!image_describer->Set_configuration_preset(stringToEnum(sFeaturePreset)))
      {
        OPENMVG_LOG_ERROR << "Preset configuration failed.";
        return EXIT_FAILURE;
      }
    }

    // 导出使用过的图像描述器和区域类型：
    // - 动态未来区域计算和/或加载
    {
      // 保存描述器的配置
      std::ofstream stream(sImage_describer.c_str());
      if (!stream)
        return EXIT_FAILURE;

      cereal::JSONOutputArchive archive(stream);
      archive(cereal::make_nvp("image_describer", image_describer));//将图像描述器的状态序列化为JSON格式并存储在文件中
      auto regionsType = image_describer->Allocate();
      archive(cereal::make_nvp("regions_type", regionsType));
    }
  }

  // 特征提取例程
  // 对于SfM_Data容器的每个视图：
  // - 如果区域文件继续存在，
  // - 如果没有文件，则计算特征
  {
    
    system::Timer timer;//系统计时器初始化
    Image<unsigned char> imageGray;//灰度图像初始化
    // 进度条初始化
    system::LoggerProgress my_progress_bar(sfm_data.GetViews().size(), "- EXTRACT FEATURES -" );

    //使用布尔值跟踪是否必须停止特征提取
    std::atomic<bool> preemptive_exit(false);
    
    // 特征提取循环
#ifdef OPENMVG_USE_OPENMP
    const unsigned int nb_max_thread = omp_get_max_threads();//获取系统可用的最大线程数

    //考虑用户指定线程数的问题
    if (iNumThreads > 0) {
        omp_set_num_threads(iNumThreads);
    } else {
        omp_set_num_threads(nb_max_thread);
    }

    //告知编译器接下来的for循环应该并行执行，并使用动态调度策略
    #pragma omp parallel for schedule(dynamic) if (iNumThreads > 0) private(imageGray)
#endif
    //循环遍历sfm_data.views中的每个视图
    for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
    {
      Views::const_iterator iterViews = sfm_data.views.begin();//声明常量迭代器，并初始化为sfm_data.views开始的位置
      std::advance(iterViews, i);//访问sfm_data.views中的第i个视图
      const View * view = iterViews->second.get();//获取第i个视图对应的View对象的指针
      const std::string
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
        sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),//创建特征文件sFeat
        sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");//创建描述符文件sDesc

      // 对于每个视图，如果特征或描述符文件不存在，则计算它们
      if (!preemptive_exit && (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc)))
      {
        if (!ReadImage(sView_filename.c_str(), &imageGray))//使用ReadImage函数读取图像，并将其存储在imageGray中
          continue;

        //
        // 查看是否有遮挡特征mask
        //
        Image<unsigned char> * mask = nullptr; //mask默认为null值

        const std::string
          mask_filename_local =
            stlplus::create_filespec(sfm_data.s_root_path,
              stlplus::basename_part(sView_filename) + "_mask", "png"),
          mask_filename_global =
            stlplus::create_filespec(sfm_data.s_root_path, "mask", "png");

        Image<unsigned char> imageMask;
        // 尝试读取本地mask（与当前视图关联的mask）
        if (stlplus::file_exists(mask_filename_local))
        {
          if (!ReadImage(mask_filename_local.c_str(), &imageMask))
          {
            OPENMVG_LOG_ERROR
              << "Invalid mask: " << mask_filename_local << ';'
              << "Stopping feature extraction.";
            preemptive_exit = true;
            continue;
          }
          // 仅当本地mask适合当前图像大小时才使用它
          if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
            mask = &imageMask;
        }
        else//如果本地mask不存在
        {
          // 尝试读取全局mask
          if (stlplus::file_exists(mask_filename_global))
          {
            if (!ReadImage(mask_filename_global.c_str(), &imageMask))
            {
              OPENMVG_LOG_ERROR
                << "Invalid mask: " << mask_filename_global << ';'
                << "Stopping feature extraction.";
              preemptive_exit = true;
              continue;
            }
            // 仅当全局mask适合当前图像大小时才使用它
            if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
              mask = &imageMask;
          }
        }

        //计算特征和描述符并将其导出到文件
        auto regions = image_describer->Describe(imageGray, mask);//特征描述
        //检查有效特征值是否被提取并保存到sFeat和sDesc文件中
        if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
          OPENMVG_LOG_ERROR
            << "Cannot save regions for image: " << sView_filename << ';'
            << "Stopping feature extraction.";
          preemptive_exit = true;
          continue;
        }
      }
      ++my_progress_bar;//更新进度条
    }
    OPENMVG_LOG_INFO << "Task done in (s): " << timer.elapsed();
  }
  return EXIT_SUCCESS;
}
