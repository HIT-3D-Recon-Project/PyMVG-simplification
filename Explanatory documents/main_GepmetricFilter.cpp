// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2019 Pierre MOULON, Romuald PERROT

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

/*
功能：
1、对上一步输出的匹配得到的特征和匹配做一下几何校验过滤，默认的方式是基础矩阵 F

输入参数：
- i // 输入路径
- o // 输出路径
- m // 上一步得到的匹配的路径，也就是未过滤的路径
- p // 输入的pair 未知
- s // 输出的pair 未知
- g // 采用几何验证的模式  默认是 f: fundamenta matrix 
*/


#include "openMVG/features/akaze/image_describer_akaze.hpp"//引入AKAZE特征描述器
#include "openMVG/features/descriptor.hpp"//引入描述子的类
#include "openMVG/features/feature.hpp"//引入特征处理相关功能
#include "openMVG/graph/graph.hpp"//图处理库
#include "openMVG/graph/graph_stats.hpp"//图统计库
#include "openMVG/matching/indMatch.hpp"//独立匹配库
#include "openMVG/matching/indMatch_utils.hpp"//独立匹配工具库
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"//对偶邻接显示库
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"//级联哈希匹配器
#include "openMVG/matching_image_collection/E_ACRobust.hpp"//一种AC-RANSAC几何过滤策略鲁棒性算法
#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"//AC-RANSAC几何过滤策略角度鲁棒性算法
#include "openMVG/matching_image_collection/Eo_Robust.hpp"//一种鲁棒性算法
#include "openMVG/matching_image_collection/F_ACRobust.hpp"//一种AC-RANSAC几何过滤策略鲁棒性算法
#include "openMVG/matching_image_collection/GeometricFilter.hpp"//几何滤波的对象（结构体）
#include "openMVG/matching_image_collection/H_ACRobust.hpp"//一种AC-RANSAC几何过滤策略鲁棒性算法
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"// 区域匹配器
#include "openMVG/matching_image_collection/Pair_Builder.hpp"// 对构建器
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"//SFM特征点提供者
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"// SFM区域提供者
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"// SFM区域提供者缓存
#include "openMVG/sfm/sfm_data.hpp"//SFM数据
#include "openMVG/sfm/sfm_data_io.hpp"//SFM数据IO
#include "openMVG/stl/stl.hpp"// STL扩展
#include "openMVG/system/timer.hpp"//系统计时器

#include "third_party/cmdLine/cmdLine.h"//第三方命令行解析
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"//第三方简化文件系统

#include <cstdlib>
#include <iostream>
#include <locale>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;

enum EGeometricModel//几何模型枚举，用于特征匹配和相机姿态估计
{
  FUNDAMENTAL_MATRIX       = 0,//基础矩阵，估计两个视图之间的相对运动
  ESSENTIAL_MATRIX         = 1,//本质矩阵，估计两个视图之间的相对旋转和平移
  HOMOGRAPHY_MATRIX        = 2,//单应性矩阵，用于处理近似平面变换的两个视图
  ESSENTIAL_MATRIX_ANGULAR = 3,//角度本质矩阵，用于相机姿态优化
  ESSENTIAL_MATRIX_ORTHO   = 4,//正交本质矩阵，当两个视图之间的旋转是正交变换时的本质矩阵
  ESSENTIAL_MATRIX_UPRIGHT = 5//竖直本质矩阵，当相机姿态相对于世界坐标系是“竖直”时的本质矩阵
};

/// 计算一系列视图之间的对应特征：
/// - 加载视图图像描述（区域：特征和描述符）
/// - 计算假定的局部特征匹配（描述符匹配）
/// - 计算几何相干特征匹配（根据假定匹配进行稳健模型估计）
/// - 导出计算数据
int main( int argc, char** argv )
{
  CmdLine cmd;//定义命令行参数

  // 场景
  std::string sSfM_Data_Filename;
  // 输入匹配
  std::string sPutativeMatchesFilename;
  // 输出匹配
  std::string sFilteredMatchesFilename;
  // 输入对
  std::string sInputPairsFilename;
  // 输出对
  std::string sOutputPairsFilename;

  //参数初始化
  std::string  sGeometricModel   = "f";//默认使用基础矩阵模型进行过滤
  bool         bForce            = false;//默认不强制执行某些操作
  bool         bGuided_matching  = false;//默认关闭引导匹配
  int          imax_iteration    = 2048;//默认AC-RANSAC算法迭代最大次数为2048
  unsigned int ui_max_cache_size = 0;//初始化进行几何滤波的匹配对数量

  //添加命令行选项
  //必需选项
  cmd.add( make_option( 'i', sSfM_Data_Filename, "input_file" ) );
  cmd.add( make_option( 'o', sFilteredMatchesFilename, "output_file" ) );//指定输入和输出的文件路径
  cmd.add( make_option( 'm', sPutativeMatchesFilename, "matches" ) ); //指定输入的特征匹配文件的路径
  // 可选选项
  cmd.add( make_option( 'p', sInputPairsFilename, "input_pairs" ) );
  cmd.add( make_option( 's', sOutputPairsFilename, "output_pairs" ) );//指定输入输出图像对的路径
  cmd.add( make_option( 'g', sGeometricModel, "geometric_model" ) );//指定用于特征匹配的几何模型
  cmd.add( make_option( 'f', bForce, "force" ) );//是否强制执行
  cmd.add( make_option( 'r', bGuided_matching, "guided_matching" ) );//是否启动几何滤波后，引导匹配
  cmd.add( make_option( 'I', imax_iteration, "max_iteration" ) );//几何滤波的最大迭代次数
  cmd.add( make_option( 'c', ui_max_cache_size, "cache_size" ) );//限制固定数量的匹配对进行几何滤波

  try
  {
    if ( argc == 1 )
      throw std::string( "Invalid command line parameter." );
    cmd.process( argc, argv );
  }

  catch ( const std::string& s )
  {
    OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
                     << "[-i|--input_file]       A SfM_Data file\n"
                     << "[-m|--matches]          (Input) matches filename\n"
                     << "[-o|--output_file]      (Output) filtered matches filename\n"
                     << "\n[Optional]\n"
                     << "[-p|--input_pairs]      (Input) pairs filename\n"
                     << "[-s|--output_pairs]     (Output) filtered pairs filename\n"
                     << "[-f|--force]            Force to recompute data\n"
                     << "[-g|--geometric_model]\n"
                     << "  (pairwise correspondences filtering thanks to robust model estimation):\n"
                     << "   f: (default) fundamental matrix,\n"
                     << "   e: essential matrix,\n"
                     << "   h: homography matrix.\n"
                     << "   a: essential matrix with an angular parametrization,\n"
                     << "   u: upright essential matrix with an angular parametrization,\n"
                     << "   o: orthographic essential matrix.\n"
                     << "[-r|--guided_matching]  Use the found model to improve the pairwise correspondences.\n"
                     << "[-c|--cache_size]\n"
                     << "  Use a regions cache (only cache_size regions will be stored in memory)\n"
                     << "  If not used, all regions will be load in memory.";

    OPENMVG_LOG_INFO << s;
    return EXIT_FAILURE;
  }//错误日志

  OPENMVG_LOG_INFO << " You called : "
                   << "\n"
                   << argv[0] << "\n"
                   << "--input_file:        " << sSfM_Data_Filename << "\n"
                   << "--matches:           " << sPutativeMatchesFilename << "\n"
                   << "--output_file:       " << sFilteredMatchesFilename << "\n"
                   << "Optional parameters: "
                   << "\n"
                   << "--input_pairs        " << sInputPairsFilename << "\n"
                   << "--output_pairs       " << sOutputPairsFilename << "\n"
                   << "--force              " << (bForce ? "true" : "false") << "\n"
                   << "--geometric_model    " << sGeometricModel << "\n"
                   << "--guided_matching    " << bGuided_matching << "\n"
                   << "--cache_size         " << ((ui_max_cache_size == 0) ? "unlimited" : std::to_string(ui_max_cache_size));

  if ( sFilteredMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid output file";
    return EXIT_FAILURE;
  }
  if ( sSfM_Data_Filename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid SfM file";
    return EXIT_FAILURE;
  }
  if ( sPutativeMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid putative matche file";
    return EXIT_FAILURE;
  }//处理非法文件输入

  const std::string sMatchesDirectory = stlplus::folder_part(sPutativeMatchesFilename);

  EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;//默认使用基础矩阵

  //判断几何模型
  switch (std::tolower(sGeometricModel[0], std::locale()))
  {
  case 'f':
      eGeometricModelToCompute = FUNDAMENTAL_MATRIX;//使用基础矩阵
      break;
  case 'e':
      eGeometricModelToCompute = ESSENTIAL_MATRIX;//使用本质矩阵
      break;
  case 'h':
      eGeometricModelToCompute = HOMOGRAPHY_MATRIX;//使用单应性矩阵
      break;
  case 'a':
      eGeometricModelToCompute = ESSENTIAL_MATRIX_ANGULAR;//使用角度本质矩阵
      break;
  case 'u':
      eGeometricModelToCompute = ESSENTIAL_MATRIX_UPRIGHT;//使用正交本质矩阵
      break;
  case 'o':
      eGeometricModelToCompute = ESSENTIAL_MATRIX_ORTHO;//使用竖直本质矩阵
      break;
  default:
      OPENMVG_LOG_ERROR << "Unknown geometric model";
      return EXIT_FAILURE;
  }

  // -----------------------------
  // - 加载SfM_Data视图和内部数据
  // a. 加载假定的描述符匹配
  // [a.1] 筛选与输入对匹配的项
  // b. 假定匹配的几何滤波
  // + 导出一些统计信息
  // -----------------------------

  //---------------------------------------
  // 读取SfM场景（图像视图和内部数据）
  //---------------------------------------
  SfM_Data sfm_data;//试图加载输入的SFM文件
  if ( !Load( sfm_data, sSfM_Data_Filename, ESfM_Data( VIEWS | INTRINSICS ) ) )
  {
    OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read.";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // 加载SfM场景区域
  //---------------------------------------
  // 从图像描述符文件初始化regions_type（用于图像区域提取）
  using namespace openMVG::features;
  // 考虑image_descripter.json在matches目录中（位于sfm_data.bin下方）
  const std::string        sImage_describer = stlplus::create_filespec( sMatchesDirectory, "image_describer.json" );//使用stlplus::create_filespec函数来创建image_describer.json文件的完整路径
  std::unique_ptr<Regions> regions_type     = Init_region_type_from_file( sImage_describer );//从image_descripter.json中初始化regions_type
  if ( !regions_type )
  {
    OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // a. 计算假定的描述符匹配
  //    - 描述符匹配（根据用户方法选择）
  //    - 仅当最近邻居比率正常时才保持对应关系
  //---------------------------------------

  // 加载相应的视图区域
  std::shared_ptr<Regions_Provider> regions_provider;
  if ( ui_max_cache_size == 0 )
  {
    // 默认区域提供程序（加载并将所有区域存储在内存中）
    regions_provider = std::make_shared<Regions_Provider>();
  }
  else
  {
    // 缓存区域提供程序（按需加载和存储区域）
    regions_provider = std::make_shared<Regions_Provider_Cache>( ui_max_cache_size );
  }

  // 在命令行上显示进度：
  system::LoggerProgress progress;

  if ( !regions_provider->load( sfm_data, sMatchesDirectory, regions_type, &progress ) )
  {
    OPENMVG_LOG_ERROR << "Invalid regions.";
    return EXIT_FAILURE;
  }//加载错误

  PairWiseMatches map_PutativeMatches;//类，代表图像对之间匹配项的集合
  //---------------------------------------
  // A. 加载初始匹配项
  //---------------------------------------
  if ( !Load( map_PutativeMatches, sPutativeMatchesFilename ) )
  {
    OPENMVG_LOG_ERROR << "Failed to load the initial matches file.";
    return EXIT_FAILURE;
  }

  if ( !sInputPairsFilename.empty() )
  {
    // 加载输入对
    OPENMVG_LOG_INFO << "Loading input pairs ...";
    Pair_Set input_pairs;
    loadPairs( sfm_data.GetViews().size(), sInputPairsFilename, input_pairs );//调用函数 loadPairs 从指定的文件名中加载对

    //使用给定的对过滤匹配项
    OPENMVG_LOG_INFO << "Filtering matches with the given pairs.";
    map_PutativeMatches = getPairs( map_PutativeMatches, input_pairs ); //过滤结果存储回 map_PutativeMatches
  }

  //---------------------------------------
  // b. 假定匹配的几何滤波
  //    - 期望几何模型的逆估计
  //    - 使用反向估计阈值的上限
  //---------------------------------------

  std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
      new ImageCollectionGeometricFilter( &sfm_data, regions_provider ) );//智能指针，用于管理ImageCollectionGeometricFilter对象的生命周期

  if ( filter_ptr )
  {
    system::Timer timer;
    const double  d_distance_ratio = 0.6;//距离比率，后续用于过滤匹配项

    PairWiseMatches map_GeometricMatches; //声明一个PairWiseMatches类型的映射，用于存储经过几何过滤后的匹配项
    /*根据选择的几何模型执行过滤
     调用filter_ptr->Robust_model_estimation方法来估计所选的几何模型
     并使用map_PutativeMatches（初始匹配项）和其他参数
     过滤后的匹配项通过调用filter_ptr->Get_geometric_matches获取，并存储在map_GeometricMatches中。
    */
    
    switch ( eGeometricModelToCompute )

    {
      case HOMOGRAPHY_MATRIX:
      {
        const bool bGeometric_only_guided_matching = true;//控制Robust_model_estimation函数中是否只使用几何引导匹配
        //使用鲁棒模型估计
        filter_ptr->Robust_model_estimation(
            GeometricFilter_HMatrix_AC( 4.0, imax_iteration ),//调用AC几何滤波器
            map_PutativeMatches,//待筛选的匹配项
            bGuided_matching,//是否使用引导匹配
            bGeometric_only_guided_matching ? -1.0 : d_distance_ratio,
            &progress );
        map_GeometricMatches = filter_ptr->Get_geometric_matches();//从filter_ptr对象中获取经过筛选的几何匹配项，并将它们存储在map_GeometricMatches中
      }
      break;
      case FUNDAMENTAL_MATRIX:
      {
        filter_ptr->Robust_model_estimation(
            GeometricFilter_FMatrix_AC( 4.0, imax_iteration ),
            map_PutativeMatches,
            bGuided_matching,
            d_distance_ratio,
            &progress );
        map_GeometricMatches = filter_ptr->Get_geometric_matches();
      }
      break;
      case ESSENTIAL_MATRIX:
      {
        filter_ptr->Robust_model_estimation(
            GeometricFilter_EMatrix_AC( 4.0, imax_iteration ),
            map_PutativeMatches,
            bGuided_matching,
            d_distance_ratio,
            &progress );
        map_GeometricMatches = filter_ptr->Get_geometric_matches();

        //-- 进行额外检查，以移除重叠不良的配对
        std::vector<PairWiseMatches::key_type> vec_toRemove;//用于存储需要移除的配对的键
        /*  遍历map_GeometricMatches中的每个配对，并检查以下条件：
            1、putativePhotometricCount：这是从map_PutativeMatches中查找到的每个配对的候选匹配数量。
            2、putativeGeometricCount：这是经过几何筛选后的匹配数量。
            3、ratio：这是几何匹配数量与候选匹配数量的比例。
            4、如果putativeGeometricCount小于50或者ratio小于0.3，则这个配对被认为是不良的，并将其键添加到vec_toRemove中。
         */
        for ( const auto& pairwisematches_it : map_GeometricMatches )
        {
          const size_t putativePhotometricCount = map_PutativeMatches.find( pairwisematches_it.first )->second.size();
          const size_t putativeGeometricCount   = pairwisematches_it.second.size();
          const float  ratio                    = putativeGeometricCount / static_cast<float>( putativePhotometricCount );
          if ( putativeGeometricCount < 50 || ratio < .3f )
          {
            // 将不良匹配的键添加到vec_toRemove
            vec_toRemove.push_back( pairwisematches_it.first );
          }
        }
        //-- 遍历vec_toRemove中的每个键，并从map_GeometricMatches中移除相应的配对
        for ( const auto& pair_to_remove_it : vec_toRemove )
        {
          map_GeometricMatches.erase( pair_to_remove_it );
        }
      }
      break;
      case ESSENTIAL_MATRIX_ANGULAR:
      {
        filter_ptr->Robust_model_estimation(
          GeometricFilter_ESphericalMatrix_AC_Angular<false>(4.0, imax_iteration),//<false>可能表示使用角度滤波器时考虑匹配图像的旋转
          map_PutativeMatches, bGuided_matching, d_distance_ratio, &progress);
        map_GeometricMatches = filter_ptr->Get_geometric_matches();
      }
      break;
      case ESSENTIAL_MATRIX_UPRIGHT:
      {
        filter_ptr->Robust_model_estimation(
          GeometricFilter_ESphericalMatrix_AC_Angular<true>(4.0, imax_iteration),//<true>可能表示使用角度滤波器时无需考虑匹配图像的旋转
          map_PutativeMatches, bGuided_matching, d_distance_ratio, &progress);
        map_GeometricMatches = filter_ptr->Get_geometric_matches();
      }
      break;
      case ESSENTIAL_MATRIX_ORTHO:
      {
        filter_ptr->Robust_model_estimation(
            GeometricFilter_EOMatrix_RA( 2.0, imax_iteration ),
            map_PutativeMatches,
            bGuided_matching,
            d_distance_ratio,
            &progress );
        map_GeometricMatches = filter_ptr->Get_geometric_matches();
      }
      break;
    }

    //---------------------------------------
    //-- 导出几何过滤匹配
    //---------------------------------------
    if ( !Save( map_GeometricMatches, sFilteredMatchesFilename ) )
    {
      OPENMVG_LOG_ERROR << "Cannot save filtered matches in: " << sFilteredMatchesFilename;
      return EXIT_FAILURE;
    }

    // -- 导出几何视图图形统计信息
    graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_GeometricMatches));

    OPENMVG_LOG_INFO << "Task done in (s): " << timer.elapsed();

    //-- 导出相邻矩阵
    OPENMVG_LOG_INFO <<  "\n Export Adjacency Matrix of the pairwise's geometric matches";

    PairWiseMatchingToAdjacencyMatrixSVG( sfm_data.GetViews().size(),
                                          map_GeometricMatches,
                                          stlplus::create_filespec( sMatchesDirectory, "GeometricAdjacencyMatrix", "svg" ) );

    const Pair_Set outputPairs = getPairs( map_GeometricMatches );

    //-- 完成几何过滤后导出视图对图
    {
      std::set<IndexT> set_ViewIds;
      std::transform( sfm_data.GetViews().begin(), sfm_data.GetViews().end(), std::inserter( set_ViewIds, set_ViewIds.begin() ), stl::RetrieveKey() );
      graph::indexedGraph putativeGraph( set_ViewIds, outputPairs );
      graph::exportToGraphvizData(
          stlplus::create_filespec( sMatchesDirectory, "geometric_matches" ),
          putativeGraph );
    }

    // 写入图像对
    if ( !sOutputPairsFilename.empty() )
    {
      OPENMVG_LOG_INFO << "Saving pairs to: " << sOutputPairsFilename;
      if ( !savePairs( sOutputPairsFilename, outputPairs ) )
      {
        OPENMVG_LOG_ERROR << "Failed to write pairs file";
        return EXIT_FAILURE;
      }
    }
  }
  return EXIT_SUCCESS;
}
