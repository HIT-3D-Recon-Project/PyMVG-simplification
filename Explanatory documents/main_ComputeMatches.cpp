// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2019 Pierre MOULON, Romuald Perrot.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/graph/graph.hpp" // 图处理库
#include "openMVG/graph/graph_stats.hpp" // 图统计库
#include "openMVG/matching/indMatch.hpp" // 独立匹配库
#include "openMVG/matching/indMatch_utils.hpp" // 独立匹配工具库
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp" // 对偶邻接显示库
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp" // 级联哈希匹配器
#include "openMVG/matching_image_collection/Matcher_Regions.hpp" // 区域匹配器
#include "openMVG/matching_image_collection/Pair_Builder.hpp" // 对构建器
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp" // SFM特征提供者
#include "openMVG/sfm/pipelines/sfm_preemptive_regions_provider.hpp" // SFM先抢占区域提供者
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp" // SFM区域提供者
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp" // SFM区域提供者缓存
#include "openMVG/sfm/sfm_data.hpp" // SFM数据
#include "openMVG/sfm/sfm_data_io.hpp" // SFM数据IO
#include "openMVG/stl/stl.hpp" // STL扩展
#include "openMVG/system/timer.hpp" // 系统计时器

#include "third_party/cmdLine/cmdLine.h" // 第三方命令行解析
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp" // 第三方简化文件系统

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG; // 使用openMVG命名空间
using namespace openMVG::matching; // 使用匹配命名空间
using namespace openMVG::sfm; // 使用SFM命名空间
using namespace openMVG::matching_image_collection; // 使用图像集匹配命名空间

/// 计算一系列视图之间对应的特征：
/// - 加载视图图像描述（区域：特征和描述符）
/// - 计算假定的局部特征匹配（描述符匹配）
int main( int argc, char** argv )
{
  CmdLine cmd;

  std::string  sSfM_Data_Filename;
  std::string  sOutputMatchesFilename = "";
  float        fDistRatio             = 0.8f; // 匹配距离比
  std::string  sPredefinedPairList    = ""; // 预定义对列表
  std::string  sNearestMatchingMethod = "AUTO"; // 最近匹配方法
  bool         bForce                 = false; // 强制重新计算标志
  unsigned int ui_max_cache_size      = 0; // 最大缓存大小

  // 抢占式匹配参数
  unsigned int ui_preemptive_feature_count = 200; // 抢占式特征数
  double preemptive_matching_percentage_threshold = 0.08; // 抢占式匹配百分比阈值

  // 必需的命令行参数
  cmd.add( make_option( 'i', sSfM_Data_Filename, "input_file" ) );
  cmd.add( make_option( 'o', sOutputMatchesFilename, "output_file" ) );
  cmd.add( make_option( 'p', sPredefinedPairList, "pair_list" ) );
  // 可选命令行参数
  cmd.add( make_option( 'r', fDistRatio, "ratio" ) );
  cmd.add( make_option( 'n', sNearestMatchingMethod, "nearest_matching_method" ) );
  cmd.add( make_option( 'f', bForce, "force" ) );
  cmd.add( make_option( 'c', ui_max_cache_size, "cache_size" ) );
  // 先发制人匹配
  cmd.add( make_option( 'P', ui_preemptive_feature_count, "preemptive_feature_count") );

  try
  {
    if ( argc == 1 )
      throw std::string( "Invalid command line parameter." );
    cmd.process( argc, argv );
  }
  catch ( const std::string& s )
  {
    OPENMVG_LOG_INFO
      << "Usage: " << argv[ 0 ] << '\n'
      << "[-i|--input_file]   A SfM_Data file\n"
      << "[-o|--output_file]  Output file where computed matches are stored\n"
      << "[-p|--pair_list]    Pairs list file\n"
      << "\n[Optional]\n"
      << "[-f|--force] Force to recompute data]\n"
      << "[-r|--ratio] Distance ratio to discard non meaningful matches\n"
      << "   0.8: (default).\n"
      << "[-n|--nearest_matching_method]\n"
      << "  AUTO: auto choice from regions type,\n"
      << "  For Scalar based regions descriptor:\n"
      << "    BRUTEFORCEL2: L2 BruteForce matching,\n"
      << "    HNSWL2: L2 Approximate Matching with Hierarchical Navigable Small World graphs,\n"
      << "    HNSWL1: L1 Approximate Matching with Hierarchical Navigable Small World graphs\n"
      << "      tailored for quantized and histogram based descriptors (e.g uint8 RootSIFT)\n"
      << "    ANNL2: L2 Approximate Nearest Neighbor matching,\n"
      << "    CASCADEHASHINGL2: L2 Cascade Hashing matching.\n"
      << "    FASTCASCADEHASHINGL2: (default)\n"
      << "      L2 Cascade Hashing with precomputed hashed regions\n"
      << "     (faster than CASCADEHASHINGL2 but use more memory).\n"
      << "  For Binary based descriptor:\n"
      << "    BRUTEFORCEHAMMING: BruteForce Hamming matching,\n"
      << "    HNSWHAMMING: Hamming Approximate Matching with Hierarchical Navigable Small World graphs\n"
      << "[-c|--cache_size]\n"
      << "  Use a regions cache (only cache_size regions will be stored in memory)\n"
      << "  If not used, all regions will be load in memory."
      << "\n[Pre-emptive matching:]\n"
      << "[-P|--preemptive_feature_count] <NUMBER> Number of feature used for pre-emptive matching";

    OPENMVG_LOG_INFO << s;
    return EXIT_FAILURE;
  }

  OPENMVG_LOG_INFO << "您调用了："
            << "\n"
            << argv[ 0 ] << "\n"
            << "--input_file " << sSfM_Data_Filename << "\n"
            << "--output_file " << sOutputMatchesFilename << "\n"
            << "--pair_list " << sPredefinedPairList << "\n"
            << "可选参数："
            << "\n"
            << "--force " << bForce << "\n"
            << "--ratio " << fDistRatio << "\n"
            << "--nearest_matching_method " << sNearestMatchingMethod << "\n"
            << "--cache_size " << ((ui_max_cache_size == 0) ? "无限制" : std::to_string(ui_max_cache_size)) << "\n"
            << "--preemptive_feature_used/count " << cmd.used('P') << " / " << ui_preemptive_feature_count;
  if (cmd.used('P'))
  {
    OPENMVG_LOG_INFO << "--preemptive_feature_count " << ui_preemptive_feature_count;
  }

  if ( sOutputMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "没有设置输出文件。";
    return EXIT_FAILURE;
  }

  // -----------------------------
  // . 加载SfM数据视图和内参数据
  // . 计算假定的描述符匹配
  // + 导出一些统计数据
  // -----------------------------

  //---------------------------------------
  // 读取SfM场景（图像视图和内参数数据）
  //---------------------------------------
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    OPENMVG_LOG_ERROR << "无法读取输入的SfM_Data文件 \""<< sSfM_Data_Filename << "\"。";
    return EXIT_FAILURE;
  }
  const std::string sMatchesDirectory = stlplus::folder_part( sOutputMatchesFilename );

  //---------------------------------------
  // 加载SfM场景区域
  //---------------------------------------
  // 从图像描述文件初始化区域类型（用于图像区域提取）
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    OPENMVG_LOG_ERROR << "无效的区域类型文件：" << sImage_describer << "。";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // a. 计算假定的描述符匹配
  //    - 根据用户选择的方法进行描述符匹配
  //    - 仅在最近邻比率合适时保留对应关系
  //---------------------------------------

  // 加载对应视图区域
  std::shared_ptr<Regions_Provider> regions_provider;
  if (ui_max_cache_size == 0)
  {
    // 默认区域提供者（加载并存储所有区域在内存中）
    regions_provider = std::make_shared<Regions_Provider>();
  }
  else
  {
    // 缓存区域提供者（按需加载并存储区域）
    regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
  }
  // 如果我们使用抢占式匹配，我们会加载更少的区域：
  if (ui_preemptive_feature_count > 0 && cmd.used('P'))
  {
    regions_provider = std::make_shared<Preemptive_Regions_Provider>(ui_preemptive_feature_count);
  }

  // 在命令行上显示进度：
  system::LoggerProgress progress;

  if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress)) {
    OPENMVG_LOG_ERROR << "无法从以下位置加载视图区域：" << sMatchesDirectory << "。";
    return EXIT_FAILURE;
  }

  PairWiseMatches map_PutativeMatches;

  // 从SfM_Data视图数据构建一些别名：
  // - 将视图列为文件名和图像尺寸的向量
  std::vector<std::string>               vec_fileNames;
  std::vector<std::pair<size_t, size_t>> vec_imagesSize;
  {
    vec_fileNames.reserve(sfm_data.GetViews().size());
    vec_imagesSize.reserve(sfm_data.GetViews().size());
    for (const auto view_it : sfm_data.GetViews())
    {
      const View * v = view_it.second.get();
      vec_fileNames.emplace_back(stlplus::create_filespec(sfm_data.s_root_path,
          v->s_Img_path));
      vec_imagesSize.emplace_back(v->ui_width, v->ui_height);
    }
  }

    OPENMVG_LOG_INFO << " - 假定匹配 - ";
  // 如果匹配已经存在，重新加载它们
  if ( !bForce && ( stlplus::file_exists( sOutputMatchesFilename ) ) )
  {
    if ( !( Load( map_PutativeMatches, sOutputMatchesFilename ) ) )
    {
      OPENMVG_LOG_ERROR << "无法加载输入的匹配文件";
      return EXIT_FAILURE;
    }
    OPENMVG_LOG_INFO
      << "\t 加载了之前的结果；"
      << " #对数: " << map_PutativeMatches.size();
  }

  else // 计算假定匹配
  {
    // 根据请求的匹配方法分配正确的匹配器
    std::unique_ptr<Matcher> collectionMatcher;
    if ( sNearestMatchingMethod == "AUTO" )
    {
      if ( regions_type->IsScalar() )
      {
        OPENMVG_LOG_INFO << "使用 FAST_CASCADE_HASHING_L2 匹配器";
        collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
      }
      else if (regions_type->IsBinary())
      {
        OPENMVG_LOG_INFO << "使用 HNSWHAMMING 匹配器";
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_HAMMING));
      }
    }
    else if (sNearestMatchingMethod == "BRUTEFORCEL2")
    {
      OPENMVG_LOG_INFO << "使用 BRUTE_FORCE_L2 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_L2));
    }
    else if (sNearestMatchingMethod == "BRUTEFORCEHAMMING")
    {
      OPENMVG_LOG_INFO << "使用 BRUTE_FORCE_HAMMING 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_HAMMING));
    }
    else if (sNearestMatchingMethod == "HNSWL2")
    {
      OPENMVG_LOG_INFO << "使用 HNSWL2 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_L2));
    }
    if (sNearestMatchingMethod == "HNSWL1")
    {
      OPENMVG_LOG_INFO << "使用 HNSWL1 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_L1));
    }
    else if (sNearestMatchingMethod == "HNSWHAMMING")
    {
      OPENMVG_LOG_INFO << "使用 HNSWHAMMING 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_HAMMING));
    }
    else if (sNearestMatchingMethod == "ANNL2")
    {
      OPENMVG_LOG_INFO << "使用 ANN_L2 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, ANN_L2));
    }
    else if (sNearestMatchingMethod == "CASCADEHASHINGL2")
    {
      OPENMVG_LOG_INFO << "使用 CASCADE_HASHING_L2 匹配器";
      collectionMatcher.reset(new Matcher_Regions(fDistRatio, CASCADE_HASHING_L2));
    }
    else if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2")
    {
      OPENMVG_LOG_INFO << "使用 FAST_CASCADE_HASHING_L2 匹配器";
      collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
    }
    if (!collectionMatcher)
    {
      OPENMVG_LOG_ERROR << "无效的最近邻方法: " << sNearestMatchingMethod;
      return EXIT_FAILURE;
    }
    // 执行匹配
    system::Timer timer;
    {
      // 从匹配模式计算必须匹配的对列表：
            Pair_Set pairs;
      // 如果没有设置预定义对文件，使用穷举匹配为默认方式
      if ( sPredefinedPairList.empty() )
      {
        OPENMVG_LOG_INFO << "没有设置输入对文件。默认使用穷尽匹配。";
        const size_t NImage = sfm_data.GetViews().size();
        pairs = exhaustivePairs( NImage );
      }
      else if ( !loadPairs( sfm_data.GetViews().size(), sPredefinedPairList, pairs ) )
      {
        OPENMVG_LOG_ERROR << "无法从文件加载对：" << sPredefinedPairList << "。";
        return EXIT_FAILURE;
      }
      OPENMVG_LOG_INFO << "对#pairs进行匹配运算: " << pairs.size();
      // 对假定对进行光度匹配
      collectionMatcher->Match( regions_provider, pairs, map_PutativeMatches, &progress );

      if (cmd.used('P')) // 抢占式筛选
      {
        // 仅当匹配数超过X时保留假定匹配
        PairWiseMatches map_filtered_matches;
        for (const auto & pairwisematches_it : map_PutativeMatches)
        {
          const size_t putative_match_count = pairwisematches_it.second.size();
          const int match_count_threshold =
            preemptive_matching_percentage_threshold * ui_preemptive_feature_count;
          // TODO: 添加一个选项来保留X最佳对
          if (putative_match_count >= match_count_threshold)  {
            // 将保留该对
            map_filtered_matches.insert(pairwisematches_it);
          }
        }
        map_PutativeMatches.clear();
        std::swap(map_filtered_matches, map_PutativeMatches);
      }

      //---------------------------------------
      //-- 导出假定匹配和对
      //---------------------------------------
      if ( !Save( map_PutativeMatches, std::string( sOutputMatchesFilename ) ) )
      {
        OPENMVG_LOG_ERROR
          << "无法在以下位置保存计算出的匹配："
          << sOutputMatchesFilename;
        return EXIT_FAILURE;
      }
      // 保存对
      const std::string sOutputPairFilename =
        stlplus::create_filespec( sMatchesDirectory, "preemptive_pairs", "txt" );
      if (!savePairs(
        sOutputPairFilename,
        getPairs(map_PutativeMatches)))
      {
        OPENMVG_LOG_ERROR
          << "无法在以下位置保存计算出的匹配对："
          << sOutputPairFilename;
        return EXIT_FAILURE;
      }
    }
    OPENMVG_LOG_INFO << "任务（区域匹配）完成时间（秒）: " << timer.elapsed();
  }

  OPENMVG_LOG_INFO << "假定对数: " << map_PutativeMatches.size();

  // -- 导出假定视图图的统计信息
  graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_PutativeMatches));

  //-- 导出假定匹配的邻接矩阵
  PairWiseMatchingToAdjacencyMatrixSVG( vec_fileNames.size(),
                                        map_PutativeMatches,
                                        stlplus::create_filespec( sMatchesDirectory, "PutativeAdjacencyMatrix", "svg" ) );
  //-- 一旦计算出假定图匹配，就导出视图对图
  {
    std::set<IndexT> set_ViewIds;
    std::transform( sfm_data.GetViews().begin(), sfm_data.GetViews().end(), std::inserter( set_ViewIds, set_ViewIds.begin() ), stl::RetrieveKey() );
    graph::indexedGraph putativeGraph( set_ViewIds, getPairs( map_PutativeMatches ) );
    graph::exportToGraphvizData(
        stlplus::create_filespec( sMatchesDirectory, "putative_matches" ),
        putativeGraph );
  }

  return EXIT_SUCCESS;
}