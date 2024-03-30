// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2019 Romuald PERROT

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>

/**
 * @brief 当前可用的配对模式列表
 *
 */
enum EPairMode
{
  PAIR_EXHAUSTIVE = 0, // 构建所有可能的图像配对
  PAIR_CONTIGUOUS = 1  // 仅连续图像配对（对于video mode很有用）
};

using namespace openMVG;
using namespace openMVG::sfm;

void usage( const char* argv0 )
{
  std::cerr << "用法: " << argv0 << '\n'
            << "[-i|--input_file]         SfM_Data 文件\n"//指定输入的SfM_Data文件，包含了场景的结构和图像信息。
            << "[-o|--output_file]        存储配对的输出文件\n"//指定存储配对输出的文件路径。
            << "\n[可选]\n"
            << "[-m|--pair_mode] mode     配对生成模式\n"
            << "       EXHAUSTIVE:        构建所有可能的配对。[默认]\n"
            << "       CONTIGUOUS:        为连续图像构建配对（与 --contiguous_count 参数一起使用）\n"
            << "[-c|--contiguous_count] X 连续链接的数量\n"
            << "       X: 将匹配0与(1->X)、...]\n"
            << "       2: 将匹配0与(1,2)，1与(2,3)，...\n"
            << "       3: 将匹配0与(1,2,3)，1与(2,3,4)，...\n"
            << std::endl;
}

// 此可执行文件计算要匹配的图像对
int main( int argc, char** argv )
{
  CmdLine cmd;

  std::string sSfMDataFilename;
  std::string sOutputPairsFilename;
  std::string sPairMode        = "EXHAUSTIVE";
  int         iContiguousCount = -1;

  // 必要元素：
  cmd.add( make_option( 'i', sSfMDataFilename, "input_file" ) );
  cmd.add( make_option( 'o', sOutputPairsFilename, "output_file" ) );
  // 可选元素：
  cmd.add( make_option( 'm', sPairMode, "pair_mode" ) );
  cmd.add( make_option( 'c', iContiguousCount, "contiguous_count" ) );

  try
  {
    if ( argc == 1 )
      throw std::string( "无效的命令行参数。" );
    cmd.process( argc, argv );
  }
  catch ( const std::string& s )
  {
    usage( argv[ 0 ] );
    std::cerr << "[错误] " << s << std::endl;

    return EXIT_FAILURE;
  }

  // 0. 解析参数
  std::cout << " 您调用了:\n"
            << argv[ 0 ] << "\n"
            << "--input_file       : " << sSfMDataFilename << "\n"
            << "--output_file      : " << sOutputPairsFilename << "\n"
            << "可选参数\n"
            << "--pair_mode        : " << sPairMode << "\n"
            << "--contiguous_count : " << iContiguousCount << "\n"
            << std::endl;

  if ( sSfMDataFilename.empty() )
  {
    usage( argv[ 0 ] );
    std::cerr << "[错误] 未设置输入文件。" << std::endl;
    exit( EXIT_FAILURE );
  }
  if ( sOutputPairsFilename.empty() )
  {
    usage( argv[ 0 ] );
    std::cerr << "[错误] 未设置输出文件。" << std::endl;
    exit( EXIT_FAILURE );
  }

  EPairMode pairMode;
  if ( sPairMode == "EXHAUSTIVE" )
  {
    pairMode = PAIR_EXHAUSTIVE;
  }
  else if ( sPairMode == "CONTIGUOUS" )
  {
    if ( iContiguousCount == -1 )
    {
      usage( argv[ 0 ] );
      std::cerr << "[错误] 选择了连续配对模式，但未设置 contiguous_count。" << std::endl;
      exit( EXIT_FAILURE );
    }

    pairMode = PAIR_CONTIGUOUS;
  }

  // 1. 加载 SfM 数据场景
  std::cout << "加载场景.";
  SfM_Data sfm_data;
  if ( !Load( sfm_data, sSfMDataFilename, ESfM_Data( VIEWS | INTRINSICS ) ) )
  {
    std::cerr << std::endl
              << "无法读取输入的 SfM_Data 文件 \"" << sSfMDataFilename << "\"。" << std::endl;
    exit( EXIT_FAILURE );
  }
  const size_t NImage = sfm_data.GetViews().size();

  // 2. 计算配对
  std::cout << "计算配对." << std::endl;
  Pair_Set pairs;
  switch ( pairMode )
  {
    case PAIR_EXHAUSTIVE:
    {
      pairs = exhaustivePairs( NImage );
      break;
    }
    case PAIR_CONTIGUOUS:
    {
      pairs = contiguousWithOverlap( NImage, iContiguousCount );
      break;
    }
    default:
    {
      std::cerr << "未知的配对模式" << std::endl;
      exit( EXIT_FAILURE );
    }
  }

  // 3. 保存配对
  std::cout << "保存配对." << std::endl;
  if ( !savePairs( sOutputPairsFilename, pairs ) )
  {
    std::cerr << "无法将配对保存到文件: \"" << sOutputPairsFilename << "\"" << std::endl;
    exit( EXIT_FAILURE );
  }

  return EXIT_SUCCESS;
}