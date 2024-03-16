// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2021 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

/*
    本文件主要依赖于gpt4与codegeex的翻译+人工整理理解
    翻译内容应该是没问题的
    译者读过一遍后对各段代码的功能大体是理解了，抽象层面的理解
    里面的各种具体的方法操作实际还是不是很懂，到时候寄希望于只照搬调用它们了
    浅层抽象止步

    翻译为python又是一苦差，还有多种不同的翻译手段
    希望到时候我会记得提醒小组讨论这回事，
    现在先休息会，这样如上

    (翻译后代码量翻了3倍，千五百行)
*/

/*
    这段代码是一个C++实现的命令行程序，用于对一组图像进行结构光流（SfM）重建。
    该程序支持不同的SfM引擎类型，如增量、全局和恒星。它还支持不同的相机模型和三角测量方法。
    用户可以指定各种选项来控制重建过程，如使用运动先验、校验内参和外参参数，以及选择初始对进行重建。

    要运行此程序，您需要提供以下命令行参数：

    -i 或 --input_file：SfM_Data场景文件的路径。
    -m 或 --match_dir：与提供的SfM_Data场景对应的匹配文件的路径。
    -o 或 --output_dir：输出数据将存储的路径。
    -s 或 --sfm_engine：用于重建的SfM引擎类型。
    -f 或 --refine_intrinsic_config：内参参数优化选项。
    -e 或 --refine_extrinsic_config：外参参数优化选项。
    -P 或 --prior_usage：启用使用运动先验（例如GPS位置）。
    -a 或 --initial_pair_a：初始对的第一个图像的文件名（不带路径）。
    -b 或 --initial_pair_b：初始对的第二个图像的文件名（不带路径）。  
    -c 或 --camera_model：未知内参视图的相机模型类型。
    --triangulation_method：三角测量方法（默认：LINFINITY_ANGULAR）。
    --resection_method：重定位/姿态估计方法（默认：P3P_KE_CVPR17）。
    -S 或 --sfm_initializer：选择SfM初始化方法。
    -G 或 --graph_simplification：图简化方法（默认：MST_X）。
    -g 或 --graph_simplification_value：图简化时要删除的边数（默认：5）。
    程序然后执行SfM重建并将结果保存到指定的输出目录。
*/

/*
    大体的流程：
    1.定义一些全局变量，如配置文件路径、图像描述文件路径等。
    2.定义一个名为main的函数，该函数接受两个参数：命令行参数的个数（argc）和命令行参数的指针（argv）。
    3.检查命令行参数是否正确（至少需要一个配置文件路径参数）。
    4.加载配置文件，并获取所需的参数，如旋转平均方法、平移平均方法、图形简化方法等。
    5.检查输入的SfM_Data文件是否存在。
    6.初始化图像描述文件（用于图像区域提取）。
    7.读取图像特征点和匹配关系。
    8.处理初始对参数（如果提供）。
    9.创建SfM引擎（根据配置文件中的重建引擎类型）。
    10.设置内参和外参的 refinement 类型。
    11.设置是否使用运动先验。
    12.执行三维重建过程。
    13.计算重建所需的时间，并生成一个HTML报告展示重建结果。
    14.将计算得到的三维场景数据和可查看的结果导出到磁盘。
    15.如果重建失败，返回失败状态。

    总之，这段代码的主要目的是从命令行参数中读取配置文件，然后执行三维重建。在执行过程中，会检查输入文件是否存在，
    初始化图像描述文件，读取图像特征点和匹配关系，处理初始对参数，创建SfM引擎，设置内参和外参的 refinement 类型，
    设置是否使用运动先验，执行三维重建过程，计算重建所需的时间，并生成一个HTML报告展示重建结果。最后，将重建得到的数据和结果导出到磁盘。
*/

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

// SfM data and its providers
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/types.hpp"

// SfM Engines
  #include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM2.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerMaxPair.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerStellar.hpp"
#include "openMVG/sfm/pipelines/stellar/sfm_stellar_engine.hpp"
#include "openMVG/sfm/pipelines/stellar/sfm_stellar_engine.hpp"

//! @brief Define the command line options for the SfM application
//! @author N. Canard
//! @author A. Chabot-Leclerc
//! @author A. Jain
//! @author A. Naveen
//! @author R. Yilmaz
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

/*
这段代码定义了三个枚举类型：ESfMSceneInitializer、ESfMEngine 和 EGraphSimplification。
这些枚举类型用于指定 SfM 场景的初始化方法、SfM 引擎类型和图简化方法。

1.ESfMSceneInitializer：用于指定 SfM 场景的初始化方法。有四个枚举值：INITIALIZE_EXISTING_POSES（初始化已存在的位姿）
  、INITIALIZE_MAX_PAIR（初始化最大匹配对）、INITIALIZE_AUTO_PAIR（初始化自动匹配对）和 INITIALIZE_STELLAR（初始化星云匹配对）。
2.ESfMEngine：用于指定 SfM 引擎类型。有五个枚举值：INCREMENTAL（增量式 SfM）、INCREMENTALV2（增量式 SfMv2）、GLOBAL（全局式 SfM）
  、STELLAR（星云式 SfM）和 INCREMENTALV2（增量式 SfMv2）。
3.EGraphSimplification：用于指定图简化方法。有两个枚举值：NONE（不进行图简化）和MST_X（使用最小生成树进行图简化）。
  此外，还定义了两个函数：StringToEnum 和 StringToEnum_EGraphSimplification。这两个函数用于将字符串映射到相应的枚举类型。
  StringToEnum 函数针对 ESfMEngine 和 ESfMSceneInitializer 枚举类型，而 StringToEnum_EGraphSimplification 函数针对 EGraphSimplification 枚举类型。
*/


// 定义枚举类型ESfMSceneInitializer，用于指定初始化SfM场景的方式
enum class ESfMSceneInitializer
{
  //! 初始化已存在的位姿
  INITIALIZE_EXISTING_POSES,
  //! 初始化最大匹配对
  INITIALIZE_MAX_PAIR,
  //! 初始化自动匹配对
  INITIALIZE_AUTO_PAIR,
  //! 初始化星云匹配对
  INITIALIZE_STELLAR
};

enum class ESfMSceneInitializer
// 定义枚举类型ESfMEngine，用于指定SfM引擎类型
enum class ESfMEngine
{
  //初始化已存在的相机姿态
  INITIALIZE_EXISTING_POSES,
  //初始化最大匹配数
  INITIALIZE_MAX_PAIR,
  //基于自动对齐初始化匹配
  INITIALIZE_AUTO_PAIR,
  //基于天体坐标初始化匹配
  INITIALIZE_STELLAR
  //! 增量式SfM
  //INCREMENTAL,
  //! 增量式SfMv2
  //INCREMENTALV2,
  //! 全局式SfM
  //GLOBAL,
  //! 星云式SfM
  //STELLAR
  //! 增量式SfMv2
  //INCREMENTALV2,
  //! 全局式SfM
  //GLOBAL,
  //! 星云式SfM
  //STELLAR
};

// 定义枚举类型ESfMEngine，用于指定SfM引擎类型
enum class ESfMEngine
{
  //! 增量式SfM
  INCREMENTAL,
  //! 增量式SfMv2
  INCREMENTALV2,
  //! 全局式SfM
  GLOBAL,
  //! 星云式SfM
  STELLAR
};

enum class ESfMEngine
{
  // Incremental mode for SfM.
  INCREMENTAL,
  // Incremental mode for SfM with new features.
  INCREMENTALV2,
  // Global mode for SfM.
  GLOBAL,
  // Stellar mode for SfM.
  STELLAR
};
/*
    StringToEnum 函数 (针对 ESfMEngine)
    目的: 将字符串映射到 ESfMEngine 枚举类型，用于识别和选择SfM引擎的类型。
    输入: 一个字符串 (str) 和一个 ESfMEngine 类型的引用 (sfm_engine)。
    输出: 如果成功找到匹配，则返回 true 并设置 sfm_engine 的值；如果未找到匹配，则返回 false。
    作用机制: 通过在一个映射表中查找字符串，该映射表定义了字符串与 ESfMEngine 枚举值之间的对应关系。
    这个函数允许程序基于用户输入的字符串（如命令行参数）来选择SfM引擎。
*/
bool StringToEnum
(
  const std::string & str,
  ESfMEngine & sfm_engine
)
{
  // 定义一个字符串到枚举类型的映射
  const std::map<std::string, ESfMEngine> string_to_enum_mapping =
  {
    {"INCREMENTAL", ESfMEngine::INCREMENTAL},
    {"INCREMENTALV2", ESfMEngine::INCREMENTALV2},
    {"GLOBAL", ESfMEngine::GLOBAL},
    {"STELLAR", ESfMEngine::STELLAR},
  };
  // 查找映射中是否有str
  const auto it  = string_to_enum_mapping.find(str);
  if (it == string_to_enum_mapping.end())
    return false;
  // 将映射中的值赋值给sfm_engine
  sfm_engine = it->second;
  return true;
}

/*
    StringToEnum 函数 (针对 ESfMSceneInitializer)
    目的: 类似于针对 ESfMEngine 的 StringToEnum 函数，这个版本将字符串映射到 ESfMSceneInitializer 枚举类型，用于选择场景初始化方法。
    输入: 一个字符串 (str) 和一个 ESfMSceneInitializer 类型的引用 (scene_initializer)。
    输出: 如果找到匹配，则返回 true 并设置 scene_initializer 的值；否则返回 false。
    作用机制: 利用一个映射表来查找和设置场景初始化方法的枚举值。这个
*/
bool StringToEnum
(
  const std::string & str,
  ESfMSceneInitializer & scene_initializer
)
{
  // 定义一个字符串到枚举类型的映射
  /*
    EXISTING_POSE：表示使用现有的姿态（即相机位置和姿态）进行场景初始化。
    MAX_PAIR：表示使用最大成对匹配（即成对匹配中匹配点对数量最多的两个图像）进行场景初始化。
    AUTO_PAIR：表示使用自动成对匹配（即根据图像特征点之间的匹配关系自动选择成对图像）进行场景初始化。
    STELLAR：表示使用星形场景初始化（即根据图像中的关键点进行场景初始化）
  */
  const std::map<std::string, ESfMSceneInitializer> string_to_enum_mapping =
  {
    {"EXISTING_POSE", ESfMSceneInitializer::INITIALIZE_EXISTING_POSES},
    {"MAX_PAIR", ESfMSceneInitializer::INITIALIZE_MAX_PAIR},
    {"AUTO_PAIR", ESfMSceneInitializer::INITIALIZE_AUTO_PAIR},
    {"STELLAR", ESfMSceneInitializer::INITIALIZE_STELLAR},
  };
  // 查找映射中是否存在str
  const auto it  = string_to_enum_mapping.find(str);
  if (it == string_to_enum_mapping.end())
    return false;
  // 将映射中的值赋值给scene_initializer
  scene_initializer = it->second;
  return true;
}

bool StringToEnum_EGraphSimplification
(
  const std::string & str,
  EGraphSimplification & graph_simplification
)
{
  // 定义一个字符串到枚举类型的映射
  /*
    表示图简化算法中的不同策略
  */
  const std::map<std::string, EGraphSimplification> string_to_enum_mapping =
  {
    {"NONE", EGraphSimplification::NONE},
    {"MST_X", EGraphSimplification::MST_X},
    {"STAR_X", EGraphSimplification::STAR_X},
  };
  // 查找映射中是否存在str
  auto it = string_to_enum_mapping.find(str);
  if (it == string_to_enum_mapping.end())
    return false;
  // 将映射中的值赋值给graph_simplification
  graph_simplification = it->second;
  return true;
}

/// From 2 given image filenames, find the two corresponding index in the View list
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex)
{
    // Check if the initial pair name is valid
    //根据给定的两个图像文件名，在视图列表中找到对应的两个索引。
    //这里可能存在理解上的误差，如果View是指图像序列，那么这个句子可能想要表达的是：从图像文件名中提取出序列号，然后在视图列表中找到对应的两个图像。
    //例如，如果图像文件名为"image1.jpg"和"image2.jpg"，那么可以从中提取出序列号1和2，然后在视图列表中找到序列号为1和2的图像
  if (initialPairName.first == initialPairName.second)
  {
    OPENMVG_LOG_ERROR << "Invalid image names. You cannot use the same image to initialize a pair.";
    return false;
  }

  // Initialize the pair index to undefined
  //将pair_index初始化为undefined
  initialPairIndex = {UndefinedIndexT, UndefinedIndexT};

  /// List views filenames and find the one that correspond to the user ones:
  //获取用户输入的图像文件名，并在视图列表中找到与这些文件名相匹配的两个视图索引
  /*
    这段C++代码的目的是从一个名为sfm_data的数据结构中查找一对图像的索引。这个数据结构可能包含多个视图（图像），
    每个视图都有一个唯一的ID和一个图像文件名。
    代码首先使用一个迭代器it遍历sfm_data中的所有视图。对于每个视图，它获取视图的文件名，
    并检查文件名是否与给定的initialPairName中的第一个或第二个图像匹配。
    如果匹配，它将视图的ID设置为initialPairIndex中的第一个或第二个元素。
    最后，代码检查initialPairIndex中的两个元素是否都已定义（即不为UndefinedIndexT）。如果都已定义，函数返回true，否则返回false。
  */
  for (Views::const_iterator it = sfm_data.GetViews().begin();
     it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    const std::string filename = stlplus::filename_part(v->s_Img_path);
    // Check if the filename matches the first image of the pair
    if (filename == initialPairName.first)
    {
      // Set the first index
      initialPairIndex.first = v->id_view;
    }
    // Check if the filename matches the second image of the pair
    else
    {
      if (filename == initialPairName.second)
      {
        // Set the second index
        initialPairIndex.second = v->id_view;
      }
    }
  }
  // Check if both indices are defined
  return (initialPairIndex.first != UndefinedIndexT &&
      initialPairIndex.second != UndefinedIndexT);
}

int main(int argc, char **argv)
{
  /*
    OPENMVG_LOG_INFO是一个日志输出宏，用于在控制台输出提示信息。<<操作符将日志信息添加到输出流中。\n表示换行符，用于在输出中插入一个新行。
    -----------------------------------------------------------是一个水平分割线，用于分隔输出中的不同部分。
    CmdLine cmd;这一行创建了一个CmdLine对象cmd，用于处理命令行参数。CmdLine是一个类，用于解析命令行参数并将其存储在cmd对象中。
    注意：CmdLine类是OpenMVG库的一部分，用于处理命令行参数。在实际应用中，可能需要根据具体需求修改或扩展CmdLine类的功能。
  */
  OPENMVG_LOG_INFO
      << "\n-----------------------------------------------------------"
      << "\n Structure from Motion:"
      << "\n-----------------------------------------------------------";
  CmdLine cmd;


  /*
    filename_sfm_data：字符串类型，表示存储SfM数据的文件名。
    directory_match：字符串类型，表示匹配图像的目录。
    filename_match：字符串类型，表示匹配图像的文件名。
    directory_output：字符串类型，表示输出目录。
    engine_name：字符串类型，表示SfM引擎的名称，默认为"INCREMENTAL"。
    sIntrinsic_refinement_options：字符串类型，表示内参优化选项，默认为"ADJUST_ALL"。
    sExtrinsic_refinement_options：字符串类型，表示外参优化选项，默认为"ADJUST_ALL"。
    b_use_motion_priors：布尔类型，表示是否使用运动先验，默认为false。
    triangulation_method：整数类型，表示三角测量方法，默认为ETriangulationMethod::DEFAULT。
    resection_method：整数类型，表示重定位方法，默认为resection::SolverType::DEFAULT。
    user_camera_model：整数类型，表示相机模型，默认为PINHOLE_CAMERA_RADIAL3。
    initial_pair_string：字符串类型，表示初始图像对。
    sfm_initializer_method：字符串类型，表示SfM初始化方法，默认为"STELLAR"。
    rotation_averaging_method：整数类型，表示旋转平均化方法，默认为ROTATION_AVERAGING_L2。
    translation_averaging_method：整数类型，表示平移平均化方法，默认为TRANSLATION_AVERAGING_SOFTL1。

    这些变量主要用于SfM相关的参数设置，包括SfM数据文件名、匹配图像目录、匹配图像文件名、输出目录、SfM引擎名称、内参优化选项、
    外参优化选项、是否使用运动先验、三角测量方法、重定位方法、相机模型、初始图像对、SfM初始化方法、旋转平均化方法和平移平均化方法
    等。
  */

  // Common options:
  std::string
      filename_sfm_data,
      directory_match,
      filename_match,
      directory_output,
      engine_name = "INCREMENTAL";

  // Bundle adjustment options:
  std::string sIntrinsic_refinement_options = "ADJUST_ALL";
  std::string sExtrinsic_refinement_options = "ADJUST_ALL";
  bool b_use_motion_priors = false;

  // Incremental SfM options
  /*
    这段代码定义了增量SfM（单目相机恢复运动结构）的选项，包括三角测量方法、重投影方法以及相机模型。

    1三角测量方法（triangulation_method）：这里使用了静态转换将枚举类型ETriangulationMethod转换为int类型，并将其默认值设置为0。
    三角测量方法用于从视图中的特征点恢复三维点。

    2重投影方法（resection_method）：这里使用了静态转换将枚举类型resection::SolverType转换为int类型，并将其默认值设置为0。
    重投影方法用于从视图中的特征点恢复相机内参矩阵和畸变系数。

    3相机模型（user_camera_model）：这里将相机模型设置为PINHOLE_CAMERA_RADIAL3，表示使用径向畸变的三元相机模型。
    相机模型用于描述相机的内参矩阵和畸变系数。

    注意：这里的代码只是定义了增量SfM的选项，实际应用中还需要根据具体情况进行设置和调整。
  */
  int triangulation_method = static_cast<int>(ETriangulationMethod::DEFAULT);
  int resection_method  = static_cast<int>(resection::SolverType::DEFAULT);
  int user_camera_model = PINHOLE_CAMERA_RADIAL3;

/*
    这段代码是C++语言编写的，主要用于设置SfM（单目视觉SLAM）的初始化对。
    SfM是一种用于从图像中恢复场景结构的算法，通常用于计算机视觉和机器人领域。

    在SfM v1中，使用一个字符串类型的pair来表示初始化对。这个pair中的第一个字符串表示第一帧图像的文件名，第二个字符串表示第二帧图像的文件名。
    初始化对用于在SLAM过程中初始化地图和相机参数。

    在SfM v2中，使用一个字符串类型的变量sfm_initializer_method来表示初始化方法。这个变量可以设置为STELLAR或IMAGE_POSE_ESTIMATION，
    分别表示使用恒星初始化方法和图像姿态估计初始化方法。初始化方法用于在SLAM过程中初始化地图和相机参数。
*/
  // SfM v1
  std::pair<std::string,std::string> initial_pair_string("","");

  // SfM v2
  std::string sfm_initializer_method = "STELLAR";

  // Global SfM
  /*
    这段代码定义了两个全局变量，分别表示旋转平均化和平移平均化的方法。

    rotation_averaging_method：旋转平均化方法，用于计算相机之间的旋转矩阵。
    这里设置为 ROTATION_AVERAGING_L2，表示使用 L2 范数进行旋转平均化。

    translation_averaging_method：平移平均化方法，用于计算相机之间的平移向量。
    这里设置为 TRANSLATION_AVERAGING_SOFTL1，表示使用软 L1 范数进行平移平均化。

    注意：这里的 ROTATION_AVERAGING_L2 和 TRANSLATION_AVERAGING_SOFTL1 是枚举类型的值，分别表示旋转平均化和平移平均化的不同方法。
    在实际应用中，可以根据具体需求选择合适的方法。
  */
  int rotation_averaging_method = int (ROTATION_AVERAGING_L2);
  int translation_averaging_method = int (TRANSLATION_AVERAGING_SOFTL1);

/*
    这段C++代码是一段命令行参数解析的代码，主要用于解析用户输入的命令行参数，并将其存储在相应的变量中。
    具体来说，这段代码定义了一些命令行参数及其对应的变量，然后使用cmd.add()函数将这些参数添加到解析器中。这些参数包括：
    通用选项：输入文件、匹配目录、匹配文件、输出目录、SfM引擎名称等。

    Bundle adjustment选项：内参 refinement 配置、外参 refinement 配置、是否使用先验等。
    Incremental SfM pipeline选项：三角测量方法、重投影方法、相机模型等。
    Incremental SfM2选项：SfM 初始化方法等。
    Incremental SfM1选项：初始对A和B等。
    Global SfM选项：旋转平均化方法、平移平均化方法等。
    Stellar SfM选项：图简化方法等。

    在解析完命令行参数后，这些变量可以用于后续的计算或操作。
*/

  // Common options
  /*
    1.添加了一个名为filename_sfm_data的命令行选项，其简短标识符为'i'，描述为"input_file"。
    这个选项用于指定输入的SFM数据文件。

    2.添加了一个名为directory_match的命令行选项，其简短标识符为'm'，描述为"match_dir"。
    这个选项用于指定用于匹配的目录。

    3.添加了一个名为filename_match的命令行选项，其简短标识符为'M'，描述为"match_file"。
    这个选项用于指定用于匹配的文件。

    4.添加了一个名为directory_output的命令行选项，其简短标识符为'o'，描述为"output_dir"。
    这个选项用于指定输出目录。

    5.添加了一个名为engine_name的命令行选项，其简短标识符为's'，描述为"sfm_engine"。
    这个选项用于指定用于SFM的引擎名称。
  */
  cmd.add( make_option('i', filename_sfm_data, "input_file") );
  cmd.add( make_option('m', directory_match, "match_dir") );
  cmd.add( make_option('M', filename_match, "match_file") );
  cmd.add( make_option('o', directory_output, "output_dir") );
  cmd.add( make_option('s', engine_name, "sfm_engine") );

  // Bundle adjustment options
  /*
    三个命令行选项：

    1.-f 或 --refine_intrinsic_config：这个选项用于指定内参矩阵的优化配置文件。
    2.-e 或 --refine_extrinsic_config：这个选项用于指定外参矩阵的优化配置文件。
    3.-P 或 --prior_usage：这个选项用于启用或禁用先验信息的使用。
    cmd.add() 函数用于向命令行解析器添加选项。make_option() 函数用于创建一个命令行选项，
    包括选项的名称、选项的简写、选项的描述以及选项的类型（例如：字符串、整数、布尔值等）。
    make_switch() 函数用于创建一个命令行开关，用于启用或禁用某个选项。

    这段代码的用途是解析命令行参数，并将它们存储在相应的变量中。注意，这些选项的类型和描述可能需要根据实际情况进行调整。
  */
  cmd.add( make_option('f', sIntrinsic_refinement_options, "refine_intrinsic_config") );
  cmd.add( make_option('e', sExtrinsic_refinement_options, "refine_extrinsic_config") );
  cmd.add( make_switch('P', "prior_usage") );

  // Incremental SfM pipeline options
  /*
    cmd是一个命令行解析对象，make_option函数用于创建一个新的命令行选项。

    't'：选项的简写，例如-t。

    triangulation_method：选项的完整名称，例如triangulation_method。

    "triangulation_method"：选项的描述，用于帮助用户理解该选项的作用。

    'r'：选项的简写，例如-r。

    resection_method：选项的完整名称，例如resection_method。

    "resection_method"：选项的描述，用于帮助用户理解该选项的作用。

    'c'：选项的简写，例如-c。

    user_camera_model：选项的完整名称，例如user_camera_model。

    "camera_model"：选项的描述，用于帮助用户理解该选项的作用。

    这些选项可能是用于控制程序的某些参数，例如选择不同的三角化方法、重投影方法或相机模型。
    用户可以通过在命令行中添加相应的选项和参数来控制程序的行为。
  */
  cmd.add( make_option('t', triangulation_method, "triangulation_method"));
  cmd.add( make_option('r', resection_method, "resection_method"));
  cmd.add( make_option('c', user_camera_model, "camera_model") );
  // Incremental SfM2
  /*
    在命令行中添加一个选项，用于指定sfm（单目视觉SLAM）初始化方法。
    在使用这个选项时，用户需要在命令行中输入 -S 或 --sfm_initializer，然后跟一个具体的初始化方法，
    例如 --sfm_initializer=mapper。程序会根据用户输入的初始化方法来选择合适的初始化方法
  */
  cmd.add( make_option('S', sfm_initializer_method, "sfm_initializer") );
  // Incremental SfM1
  /*
    这段C++代码是用于实现增量SfM（单目相机视觉SLAM）算法的部分。具体来说，这段代码是用于设置初始对（pair of images）的。

    cmd.add( make_option('a', initial_pair_string.first, "initial_pair_a") ); 
    这行代码是向命令行添加一个名为initial_pair_a的选项，其值为initial_pair_string.first。
    initial_pair_string.first是一个字符串，表示初始对中的第一张图像的名称。

    cmd.add( make_option('b', initial_pair_string.second, "initial_pair_b") ); 
    这行代码是向命令行添加一个名为initial_pair_b的选项，其值为initial_pair_string.second。
    initial_pair_string.second是一个字符串，表示初始对中的第二张图像的名称。

    增量SfM算法是一种从少量图像中恢复场景结构的算法。通过比较两张图像中的特征点，可以计算它们之间的相对位姿。
    然后，可以将这个位姿应用到所有其他图像上，从而逐步构建整个场景。

    在使用这段代码时，需要注意以下几点：

    1initial_pair_string是一个包含两个字符串的pair对象，分别表示初始对中的第一张和第二张图像的名称。

    2命令行选项-a和-b分别用于指定初始对中的第一张和第二张图像的名称。
  */
  cmd.add( make_option('a', initial_pair_string.first, "initial_pair_a") );
  cmd.add( make_option('b', initial_pair_string.second, "initial_pair_b") );
  // Global SfM
  /*
    这段C++代码是用于添加命令行选项的函数，用于指定全局SfM（单目相机三维重建）中的旋转平均化和平移平均化方法。

    cmd.add( make_option('R', rotation_averaging_method, "rotationAveraging") ); 
    这行代码添加了一个名为'R'的命令行选项，用于指定旋转平均化方法。rotation_averaging_method是一个变量，用于存储旋转平均化的方法，
    可以是L1、L2、softL1等。"rotationAveraging"是一个字符串，用于描述该选项的用途。

    cmd.add( make_option('T', translation_averaging_method, "translationAveraging") ); 
    这行代码添加了一个名为'T'的命令行选项，用于指定平移平均化方法。translation_averaging_method是一个变量，用于存储平移平均化的方法，
    可以是L1、L2、softL1等。"translationAveraging"是一个字符串，用于描述该选项的用途。

    注意，这些选项都是全局SfM中的参数，用于控制三维重建的过程。在实际使用中，可以根据具体需求和数据集来选择合适的参数。
  */
  cmd.add( make_option('R', rotation_averaging_method, "rotationAveraging") );
  cmd.add( make_option('T', translation_averaging_method, "translationAveraging") );
  // Stellar SfM
  /*
    这段C++代码是用于添加命令行参数的。cmd是一个命令行参数解析器对象，make_option函数用于创建一个新的命令行参数。

    graph_simplification是一个字符串变量，用于存储图形简化的方法。
    graph_simplification_value是一个整数变量，用于存储图形简化的参数值。

    cmd.add( make_option('G', graph_simplification, "graph_simplification") )
    这行代码添加了一个名为graph_simplification的命令行参数，其类型为字符串，并将其值存储在graph_simplification变量中。

    cmd.add( make_option('g', graph_simplification_value, "graph_simplification_value") )
    这行代码添加了一个名为graph_simplification_value的命令行参数，其类型为整数，并将其值存储在graph_simplification_value变量中。

    注意，命令行参数的名称和类型需要与实际程序的参数匹配。在实际使用中，用户可以通过命令行指定这些参数的值，例如：
    stellar_sfm -G MST_X -g 5
    这将设置graph_simplification为MST_X，graph_simplification_value为5。
  */
  std::string graph_simplification = "MST_X";
  int graph_simplification_value = 5;
  cmd.add( make_option('G', graph_simplification, "graph_simplification") );
  cmd.add( make_option('g', graph_simplification_value, "graph_simplification_value") );

/*
    这段C++代码是一个错误处理和帮助信息的输出。当用户在运行程序时提供了无效的参数，程序会抛出一个std::string类型的异常，并输出错误信息。
    代码首先检查命令行参数的数量，如果只有一个参数，则抛出异常。然后，它尝试处理命令行参数，如果发生异常，则捕获异常并输出帮助信息。
    帮助信息包括程序的用法、必选参数和可选参数的描述、特定于引擎的参数等。这有助于用户了解如何正确地使用程序，以及每个参数的作用。

    以下是代码的详细解释：
    1检查命令行参数的数量。如果只有一个参数，则抛出异常。
    2尝试处理命令行参数。如果发生异常，则捕获异常并输出帮助信息。
    3输出程序的用法。
    4输出必选参数的描述。
    5输出可选参数的描述。
    6输出特定于引擎的参数。
    7如果发生异常，输出错误信息并返回失败状态。
*/

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {

/*
    用于输出程序的用法和可选参数。OpenMVG是一个开源的计算机视觉库，用于从图像中恢复场景的3D结构。

    程序的用途是进行SfM（单张图像恢复）重建，即从一组图像中恢复场景的3D结构。可选参数包括：

    1输入文件：路径到一个SfM_Data场景，即包含图像和相机参数的数据文件。
    2匹配目录：路径到与提供的SfM_Data场景对应的匹配文件。
    3输出目录：路径到存储输出数据的目录。
    4SfM引擎：类型用于SfM重建的引擎。有四种选项：INCREMENTAL（增量）、INCREMENTALV2（增量V2）、GLOBAL（全局）和STELLAR（星系）。
    5匹配文件：路径到要使用的匹配文件（例如matches.f.txt或matches.f.bin）。
    6refined_intrinsic_config：内参参数优化选项。
     有四种选项：ADJUST_ALL（调整所有现有参数，默认）、NONE（内参参数保持不变）、ADJUST_FOCAL_LENGTH（调整焦距）
     、ADJUST_PRINCIPAL_POINT（调整主点位置）和ADJUST_DISTORTION（调整畸变系数）。
    7refined_extrinsic_config：外参参数优化选项。
     有四种选项：ADJUST_ALL（调整所有现有参数，默认）、NONE（外参参数保持不变）。
    8prior_usage：启用使用运动先验（例如GPS位置）


+中文版本
    OPENMVG_LOG_INFO << "使用方法: " << argv[0] << '\n'
      << "[必需]\n"
      << "[-i|--input_file] 指向SfM_Data场景的路径\n"
      << "[-m|--match_dir] 指向与提供的SfM_Data场景对应的匹配的路径\n"
      << "[-o|--output_dir] 输出数据将被存储的路径\n"
      << "[-s|--sfm_engine] 用于重建的SfM引擎类型\n"
      << "\t INCREMENTAL   : 以2视图种子逐个添加图像\n"
      << "\t INCREMENTALV2 : 以2或N视图种子逐个添加图像（实验性）\n"
      << "\t GLOBAL        : 全局初始化旋转和平移\n"
      << "\t STELLAR       : n-组局部运动优化 + 全局SfM\n"
      << "\n\n"
      << "[可选参数]\n"
      << "\n\n"
      << "[通用]\n"
      << "[-M|--match_file] 指向匹配文件的路径（例如 matches.f.txt 或 matches.f.bin）\n"
      << "[-f|--refine_intrinsic_config] 内参参数优化选项\n"
      << "\t ADJUST_ALL -> 优化所有现有参数（默认）\n"
      << "\t NONE -> 内参参数保持不变\n"
      << "\t ADJUST_FOCAL_LENGTH -> 优化焦距\n"
      << "\t ADJUST_PRINCIPAL_POINT -> 优化主点位置\n"
      << "\t ADJUST_DISTORTION -> 优化畸变系数（如果有）\n"
      << "\t -> 注意：选项可以组合使用，如 '|'\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      <<    "\t\t-> 优化焦距 & 主点位置\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      <<    "\t\t-> 优化焦距 & 畸变系数（如果有）\n"
      << "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
      <<    "\t\t-> 优化主点位置 & 畸变系数（如果有）\n"
      << "[-e|--refine_extrinsic_config] 外参参数优化选项\n"
      << "\t ADJUST_ALL -> 优化所有现有参数（默认）\n"
      << "\t NONE -> 外参参数保持不变\n"
      << "[-P|--prior_usage] 启用使用运动先验（即GPS位置）(默认：false)\n"
      << "\n\n"
      << "[引擎特定]\n"
      << "\n\n"
      << "[INCREMENTAL]\n"
      << "\t[-a|--initial_pair_a] 第一张图像的文件名（不带路径）\n"
      << "\t[-b|--initial_pair_b] 第二张图像的文件名（不带路径）\n"
      << "\t[-c|--camera_model] 未知内参的相机模型类型:\n"
      << "\t\t 1: Pinhole \n"
      << "\t\t 2: Pinhole radial 1\n"
      << "\t\t 3: Pinhole radial 3 (default)\n"
      << "\t\t 4: Pinhole radial 3 + tangential 2\n"
      << "\t\t 5: Pinhole fisheye\n"
      << "\t[--triangulation_method] 三角测量方法（默认=" << triangulation_method << "）:\n"
*/
    OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
      << "[Required]\n"
      << "[-i|--input_file] path to a SfM_Data scene\n"
      << "[-m|--match_dir] path to the matches that corresponds to the provided SfM_Data scene\n"
      << "[-o|--output_dir] path where the output data will be stored\n"
      << "[-s|--sfm_engine] Type of SfM Engine to use for the reconstruction\n"
      << "\t INCREMENTAL   : add image sequentially to a 2 view seed\n"
      << "\t INCREMENTALV2 : add image sequentially to a 2 or N view seed (experimental)\n"
      << "\t GLOBAL        : initialize globally rotation and translations\n"
      << "\t STELLAR       : n-uplets local motion refinements + global SfM\n"
      << "\n\n"
      << "[Optional parameters]\n"
      << "\n\n"
      << "[Common]\n"
      << "[-M|--match_file] path to the match file to use (i.e matches.f.txt or matches.f.bin)\n"
      << "[-f|--refine_intrinsic_config] Intrinsic parameters refinement option\n"
      << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      << "\t NONE -> intrinsic parameters are held as constant\n"
      << "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
      << "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
      << "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
      << "\t -> NOTE: options can be combined thanks to '|'\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      <<    "\t\t-> refine the focal length & the principal point position\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      <<    "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
      << "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
      <<    "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
      << "[-e|--refine_extrinsic_config] Extrinsic parameters refinement option\n"
      << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      << "\t NONE -> extrinsic parameters are held as constant\n"
      << "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions) (default: false)\n"
      << "\n\n"
      << "[Engine specifics]\n"
      << "\n\n"
      << "[INCREMENTAL]\n"
      << "\t[-a|--initial_pair_a] filename of the first image (without path)\n"
      << "\t[-b|--initial_pair_b] filename of the second image (without path)\n"
      << "\t[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      << "\t\t 1: Pinhole \n"
      << "\t\t 2: Pinhole radial 1\n"
      << "\t\t 3: Pinhole radial 3 (default)\n"
      << "\t\t 4: Pinhole radial 3 + tangential 2\n"
      << "\t\t 5: Pinhole fisheye\n"
      << "\t[--triangulation_method] triangulation method (default=" << triangulation_method << "):\n"
      /*
      1.triangulation_method：三角化方法。有四种方法可供选择：
        DIRECT_LINEAR_TRANSFORM：直接线性变换
        L1_ANGULAR：L1角度
        LINFINITY_ANGULAR：L无穷大角度
        INVERSE_DEPTH_WEIGHTED_MIDPOINT：反深度加权中点

    2.resection_method：重投影/姿态估计方法。有五种方法可供选择：
        DLT_6POINTS：直接线性变换 6Points | 不使用内参数据
        P3P_KE_CVPR17：P3P_KE_CVPR17
        P3P_KNEIP_CVPR11：P3P_KNEIP_CVPR11
        P3P_NORDBERG_ECCV18：P3P_NORDBERG_ECCV18
        UP2P_KUKELOVA_ACCV10：UP2P_KUKELOVA_ACCV10 | 2Points | 垂直相机
        这些选项的值是通过static_cast<int>(ETriangulationMethod::DIRECT_LINEAR_TRANSFORM)等语句转换为整数的。
        在打印时，它们将显示为[--triangulation_method]等。
      
      
      */
      << "\t\t" << static_cast<int>(ETriangulationMethod::DIRECT_LINEAR_TRANSFORM) << ": DIRECT_LINEAR_TRANSFORM\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::L1_ANGULAR) << ": L1_ANGULAR\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::LINFINITY_ANGULAR) << ": LINFINITY_ANGULAR\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::INVERSE_DEPTH_WEIGHTED_MIDPOINT) << ": INVERSE_DEPTH_WEIGHTED_MIDPOINT\n"
      << "\t[--resection_method] resection/pose estimation method (default=" << resection_method << "):\n"
      << "\t\t" << static_cast<int>(resection::SolverType::DLT_6POINTS) << ": DIRECT_LINEAR_TRANSFORM 6Points | does not use intrinsic data\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_KE_CVPR17) << ": P3P_KE_CVPR17\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_KNEIP_CVPR11) << ": P3P_KNEIP_CVPR11\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_NORDBERG_ECCV18) << ": P3P_NORDBERG_ECCV18\n"
      << "\t\t" << static_cast<int>(resection::SolverType::UP2P_KUKELOVA_ACCV10)  << ": UP2P_KUKELOVA_ACCV10 | 2Points | upright camera\n"
      << "\n\n"
      /*
        用于打印一段关于SfM（三维场景恢复）初始化方法的说明。

        具体来说，这段代码首先输出一个标题“[INCREMENTALV2]”，然后输出一段关于SfM初始化方法的选项。用户可以选择不同的初始化方法来恢复三维场景。

        选项包括：

        -S 或 --sfm_initializer：选择SfM初始化方法。有四种选项：

        EXISTING_POSE：从现有的sfm_data相机姿态中初始化重建。
        MAX_PAIR：从具有最多匹配对的pair中初始化重建。
        AUTO_PAIR：自动选择一对进行初始化。
        STELLAR：使用“stellar”重建方法进行初始化。
        -c 或 --camera_model：选择相机模型类型，用于具有未知内参的视图。有五种选项：

        1：针孔模型。
        2：针孔径向1模型。
        3：针孔径向3模型（默认）。
        4：针孔径向3 + 切向2模型。
        5：针孔鱼眼模型。
        这段代码的用途是帮助用户了解如何使用SfM初始化方法进行三维场景恢复，并选择合适的相机模型类型。
        在使用时，用户需要根据实际情况和需求选择合适的选项。

        注意事项：

        在使用这些选项时，请确保已经安装了相应的库和程序，并正确配置了环境变量。

        在使用这些选项时，请确保已经正确处理了图像数据，并生成了sfm_data文件。

        在使用这些选项时，请确保已经正确设置了相机内参和畸变参数。
      */
      << "[INCREMENTALV2]\n"
      << "\t[-S|--sfm_initializer] Choose the SfM initializer method:\n"
      << "\t\t 'EXISTING_POSE'-> Initialize the reconstruction from the existing sfm_data camera poses\n"
      << "\t\t 'MAX_PAIR'-> Initialize the reconstruction from the pair that has the most of matches\n"
      << "\t\t 'AUTO_PAIR'-> Initialize the reconstruction with a pair selected automatically\n"
      << "\t\t 'STELLAR'-> Initialize the reconstruction with a 'stellar' reconstruction\n"
      << "\t[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      << "\t\t 1: Pinhole \n"
      << "\t\t 2: Pinhole radial 1\n"
      << "\t\t 3: Pinhole radial 3 (default)\n"
      << "\t\t 4: Pinhole radial 3 + tangential 2\n"
      << "\t\t 5: Pinhole fisheye\n"
      /*
        triangulation_method：三角测量方法。有四种方法可供选择：

        DIRECT_LINEAR_TRANSFORM：直接线性变换
        L1_ANGULAR：L1角度
        LINFINITY_ANGULAR：L无穷大角度
        INVERSE_DEPTH_WEIGHTED_MIDPOINT：反深度加权中点
        resection_method：重投影/姿态估计方法。有六种方法可供选择：

        DLT_6POINTS：直接线性变换 6Points | 不使用内参数据
        P3P_KE_CVPR17：P3P_KE_CVPR17
        P3P_KNEIP_CVPR11：P3P_KNEIP_CVPR11
        P3P_NORDBERG_ECCV18：P3P_NORDBERG_ECCV18
        UP2P_KUKELOVA_ACCV10：UP2P_KUKELOVA_ACCV10 | 2Points | 垂直相机
        注意：这段代码是用于打印选项和它们的值的，而不是用于实现这些选项的功能。
        实现这些选项的功能需要使用相应的算法和数据结构。
      */
      << "\t[--triangulation_method] triangulation method (default=" << triangulation_method << "):\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::DIRECT_LINEAR_TRANSFORM) << ": DIRECT_LINEAR_TRANSFORM\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::L1_ANGULAR) << ": L1_ANGULAR\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::LINFINITY_ANGULAR) << ": LINFINITY_ANGULAR\n"
      << "\t\t" << static_cast<int>(ETriangulationMethod::INVERSE_DEPTH_WEIGHTED_MIDPOINT) << ": INVERSE_DEPTH_WEIGHTED_MIDPOINT\n"
      << "\t[--resection_method] resection/pose estimation method (default=" << resection_method << "):\n"
      << "\t\t" << static_cast<int>(resection::SolverType::DLT_6POINTS) << ": DIRECT_LINEAR_TRANSFORM 6Points | does not use intrinsic data\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_KE_CVPR17) << ": P3P_KE_CVPR17\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_KNEIP_CVPR11) << ": P3P_KNEIP_CVPR11\n"
      << "\t\t" << static_cast<int>(resection::SolverType::P3P_NORDBERG_ECCV18) << ": P3P_NORDBERG_ECCV18\n"
      << "\t\t" << static_cast<int>(resection::SolverType::UP2P_KUKELOVA_ACCV10)  << ": UP2P_KUKELOVA_ACCV10 | 2Points | upright camera\n"
      << "\n\n"
      /*
        "[GLOBAL]"：全局选项 
        a. "-R"或"--rotationAveraging"：旋转平均化 
        i. 1 -> L1最小化 ii. 2 -> L2最小化（默认） 

        b. "-T"或"--translationAveraging"：平移平均化 
        i. 1 -> L1最小化 ii. 2 -> L2最小化（默认） iii. 3 -> SoftL1最小化 iv. 4 -> LiGT：线性全局平移约束从旋转和匹配中获取
        
        "[STELLAR]"：恒星选项 
        a. "-G"或"--graph_simplification"：图简化 
        i. NONE ii. MST_X iii. STAR_X 
        b. "-g"或"--graph_simplification_value"：图简化值 
        i. 数字（默认：" << graph_simplification_value << "）
      */
      << "[GLOBAL]\n"
      << "\t[-R|--rotationAveraging]\n"
      << "\t\t 1 -> L1 minimization\n"
      << "\t\t 2 -> L2 minimization (default)\n"
      << "\t[-T|--translationAveraging]:\n"
      << "\t\t 1 -> L1 minimization\n"
      << "\t\t 2 -> L2 minimization of sum of squared Chordal distances\n"
      << "\t\t 3 -> SoftL1 minimization (default)\n"
      << "\t\t 4 -> LiGT: Linear Global Translation constraints from rotation and matches\n"
      << "[STELLAR]\n"
      << "\t[-G|--graph_simplification]\n"
      << "\t\t -> NONE\n"
      << "\t\t -> MST_X\n"
      << "\t\t -> STAR_X\n"
      << "\t[-g|--graph_simplification_value]\n"
      << "\t\t -> Number (default: " << graph_simplification_value << ")";


    OPENMVG_LOG_ERROR << s;
    return EXIT_FAILURE;
  }

/*
    这段C++代码的目的是从命令行参数中获取一个名为'P'的参数，并将其存储在变量b_use_motion_priors中。
    cmd.used('P')是一个布尔值，表示命令行参数'P'是否被使用。如果cmd.used('P')为真，则b_use_motion_priors的值为1，否则为0。
    这段代码的实现原理是通过检查命令行参数来决定是否使用运动先验。
    运动先验是指在运动模型中，假设运动模型是正确的，然后根据运动模型预测下一帧图像，从而得到当前帧图像中物体的位置和运动信息。
    使用运动先验可以提高跟踪的准确性，特别是在目标移动速度较慢或背景较为复杂的情况下。
    这段代码的用途是在目标跟踪过程中，根据命令行参数来决定是否使用运动先验。
    如果使用运动先验，则需要根据运动模型预测下一帧图像，从而得到当前帧图像中物体的位置和运动信息。
    如果不使用运动先验，则直接使用当前帧图像中的特征点进行目标跟踪。
    在使用这段代码时，需要注意以下几点：

    1确保命令行参数'P'被正确设置。如果'P'没有被设置，则cmd.used('P')将为假，b_use_motion_priors将为0，即不使用运动先验。
    2如果使用运动先验，需要确保运动模型是正确的，否则可能会导致跟踪结果不准确。
    3如果使用运动先验，需要确保当前帧图像中的特征点足够多，否则可能会导致跟踪失败。
*/

  b_use_motion_priors = cmd.used('P');

/*
    这段C++代码是在检查命令行参数的有效性。首先，它检查triangulation_method是否是一个有效的三角化方法，
    如果不是，则输出错误信息并返回失败。然后，它检查user_camera_model是否是一个有效的相机模型，如果不是，则输出错误信息并返回失败。
    isValid函数可能是用于检查给定参数是否在某个范围内或是否符合某种条件。
    这里，它可能是检查ETriangulationMethod和EINTRINSIC枚举类型的值是否在有效的范围内。
    这段代码的主要用途是确保用户提供的命令行参数是有效的，从而避免在程序运行过程中出现错误。
    需要注意的是，这段代码中的错误处理方式是直接输出错误信息并返回失败，这可能会导致程序无法继续运行。
    在实际应用中，可能需要根据具体情况进行适当的错误处理。
*/

  // Check validity of command line parameters:
  if ( !isValid(static_cast<ETriangulationMethod>(triangulation_method))) {
    OPENMVG_LOG_ERROR << "Invalid triangulation method";
    return EXIT_FAILURE;
  }

  if ( !isValid(openMVG::cameras::EINTRINSIC(user_camera_model)) )  {
    OPENMVG_LOG_ERROR << "Invalid camera type";
    return EXIT_FAILURE;
  }
/*
    这段C++代码主要用来解析输入的参数，并检查它们是否有效。

    1.首先，将字符串sIntrinsic_refinement_options转换为Intrinsic_Parameter_Type类型的变量intrinsic_refinement_options。
    Intrinsic_Parameter_Type是一个枚举类型，表示内参参数的类型。StringTo_Intrinsic_Parameter_Type是一个函数，
    用于将字符串转换为Intrinsic_Parameter_Type类型的变量。

    2.然后，检查intrinsic_refinement_options是否为0。如果是，则输出错误信息，并返回失败状态。

    3.接下来，将字符串sExtrinsic_refinement_options转换为Extrinsic_Parameter_Type类型的变量extrinsic_refinement_options。
    Extrinsic_Parameter_Type也是一个枚举类型，表示外参参数的类型。StringTo_Extrinsic_Parameter_Type是一个函数，
    用于将字符串转换为Extrinsic_Parameter_Type类型的变量。

    4.最后，检查extrinsic_refinement_options是否为0。如果是，则输出错误信息，并返回失败状态。

    总之，这段代码的作用是将输入的字符串参数转换为相应的枚举类型变量，并检查它们是否有效。如果无效，则输出错误信息，并返回失败状态。
*/
  const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
      cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
  if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0) )
  {
    OPENMVG_LOG_ERROR << "Invalid input for Bundle Adjustment Intrinsic parameter refinement option";
    return EXIT_FAILURE;
  }

  const sfm::Extrinsic_Parameter_Type extrinsic_refinement_options =
      sfm::StringTo_Extrinsic_Parameter_Type(sExtrinsic_refinement_options);
  if (extrinsic_refinement_options == static_cast<sfm::Extrinsic_Parameter_Type>(0) )
  {
    OPENMVG_LOG_ERROR << "Invalid input for the Bundle Adjustment Extrinsic parameter refinement option";
    return EXIT_FAILURE;
  }
/*
    这段代码是用于解析用户输入的参数，并检查它们是否有效。
    首先，它将用户输入的sfm_initializer_method字符串转换为scene_initializer_enum枚举类型。如果转换失败，则输出错误信息并返回失败状态。

    然后，它将用户输入的engine_name字符串转换为sfm_engine_type枚举类型。如果转换失败，则输出错误信息并返回失败状态。

    最后，它检查rotation_averaging_method是否在有效范围内。如果不在，则输出错误信息并返回失败状态。
*/
  ESfMSceneInitializer scene_initializer_enum;
  if (!StringToEnum(sfm_initializer_method, scene_initializer_enum))
  {
    OPENMVG_LOG_ERROR << "Invalid input for the SfM initializer option";
    return EXIT_FAILURE;
  }

  ESfMEngine sfm_engine_type;
  if (!StringToEnum(engine_name, sfm_engine_type))
  {
    OPENMVG_LOG_ERROR << "Invalid input for the SfM Engine type";
    return EXIT_FAILURE;
  }

  if (rotation_averaging_method < ROTATION_AVERAGING_L1 ||
      rotation_averaging_method > ROTATION_AVERAGING_L2 )  {
    OPENMVG_LOG_ERROR << "Rotation averaging method is invalid";
    return EXIT_FAILURE;
  }
/*
    这段代码是在C++代码中检查是否使用了专利 ligt 方法进行平移平均化。如果没有使用这个方法，那么会输出错误信息并返回失败状态。

    #ifndef USE_PATENTED_LIGT 是一个编译时条件编译指令，当 USE_PATENTED_LIGT 没有被定义时，下面的代码块会被执行。

    if (translation_averaging_method == TRANSLATION_LIGT) { 这一行代码检查当前使用的平移平均化方法是否为 ligt。如果是，那么会执行下面的代码块。

    OPENMVG_LOG_ERROR << "OpenMVG was not compiled with USE_PATENTED_LIGT cmake option"; 这一行代码会输出错误信息，表示 OpenMVG 没有被编译为使用 ligt 方法。

    return EXIT_FAILURE; 这一行代码会返回失败状态，表示程序执行失败。

    #endif 是一个编译时条件编译指令，当 USE_PATENTED_LIGT 被定义时，下面的代码块不会被执行。
*/
#ifndef USE_PATENTED_LIGT
  if (translation_averaging_method == TRANSLATION_LIGT) {
    OPENMVG_LOG_ERROR << "OpenMVG was not compiled with USE_PATENTED_LIGT cmake option";
    return EXIT_FAILURE;
  }
#endif

/*
    这段代码是用于检查用户输入的参数是否合法，并设置相应的变量。

    1首先，检查translation_averaging_method是否在TRANSLATION_AVERAGING_L1和TRANSLATION_LIGT之间。如果不在这个范围内，输出错误信息并返回失败。

    2然后，将graph_simplification字符串转换为EGraphSimplification枚举类型，并检查是否成功。如果不成功，输出错误信息并返回失败。

    3检查graph_simplification_value是否大于1，如果不大于1，输出错误信息并返回失败。

    4检查directory_output是否为空，如果为空，输出错误信息并返回失败。

    总之，这段代码的作用是检查用户输入的参数是否合法，并设置相应的变量。如果参数不合法，输出错误信息并返回失败
*/
  if (translation_averaging_method < TRANSLATION_AVERAGING_L1 ||
      translation_averaging_method > TRANSLATION_LIGT )  {
    OPENMVG_LOG_ERROR << "Translation averaging method is invalid";
    return EXIT_FAILURE;
  }

  EGraphSimplification graph_simplification_method;
  if (!StringToEnum_EGraphSimplification(graph_simplification, graph_simplification_method))
  {
    OPENMVG_LOG_ERROR << "Cannot recognize graph simplification method";
    return EXIT_FAILURE;
  }
  if (graph_simplification_value <= 1)
  {
    OPENMVG_LOG_ERROR << "graph simplification value must be > 1";
    return EXIT_FAILURE;
  }

  if (directory_output.empty())  {
    OPENMVG_LOG_ERROR << "It is an invalid output directory";
    return EXIT_FAILURE;
  }

  // SfM related

  // Load input SfM_Data scene
  /*
    这段C++代码是OpenMVG库中的一个片段，主要用于读取和处理三维场景数据（SfM_Data）。以下是代码的详细解释：

    1定义一个名为sfm_data的SfM_Data对象，用于存储三维场景数据。

    2定义一个枚举类型ESfM_Data，用于表示要加载的SfM_Data对象的类型。这里定义了两种类型：VIEWS（视图）和INTRINSICS（内参），
    以及它们的组合VIEWS|INTRINSICS（视图和内参）。

    3判断scene_initializer_enum的值，如果为ESfMSceneInitializer::INITIALIZE_EXISTING_POSES，则加载ESfM_Data类型为VIEWS|INTRINSICS|EXTRINSICS的数据；
    否则，加载ESfM_Data类型为VIEWS|INTRINSICS的数据。

    4使用Load函数读取SfM_Data文件filename_sfm_data，并将结果存储在sfm_data对象中。如果读取失败，输出错误信息并返回失败标志。

    5判断directory_output是否存在，如果不存在，则尝试创建该目录。如果创建失败，输出错误信息并返回失败标志。

    总之，这段代码的主要作用是读取一个SfM_Data文件，并根据指定的类型加载数据。同时，确保输出目录存在。
  */
  SfM_Data sfm_data;
  const ESfM_Data sfm_data_loading_etypes =
      scene_initializer_enum == ESfMSceneInitializer::INITIALIZE_EXISTING_POSES ?
        ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS) : ESfM_Data(VIEWS|INTRINSICS);
  if (!Load(sfm_data, filename_sfm_data, sfm_data_loading_etypes)) {
    OPENMVG_LOG_ERROR << "The input SfM_Data file \""<< filename_sfm_data << "\" cannot be read.";
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(directory_output))
  {
    if (!stlplus::folder_create(directory_output))
    {
      OPENMVG_LOG_ERROR << "Cannot create the output directory";
      return EXIT_FAILURE;
    }
  }

  //
  // Match and features
  //
  /*
    这段C++代码的目的是从给定的文件名中提取文件夹路径和文件名，并将它们分别存储在directory_match和filename_match中。

    1首先，检查directory_match是否为空，filename_match是否不为空，并且文件是否存在。
    2如果满足条件，则使用stlplus::folder_part()函数从filename_match中提取文件夹路径，并将其存储在directory_match中。
    3使用stlplus::filename_part()函数从filename_match中提取文件名，并将其存储在filename_match中。
    注意：stlplus是一个C++库，用于处理文件和目录。folder_part()函数用于从给定的路径中提取文件夹部分，filename_part()函数用于从给定的路径中提取文件名部分。
  */
  if (directory_match.empty() && !filename_match.empty() && stlplus::file_exists(filename_match))
  {
    directory_match = stlplus::folder_part(filename_match);
    filename_match = stlplus::filename_part(filename_match);
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  /*
  这段C++代码是用于从文件中读取图像描述符（image_describer）的类型，并将其转换为Regions类型的智能指针。以下是代码的详细解释：

    1使用using namespace openMVG::features;声明使用openMVG::features命名空间，以便使用其中的类和函数。

    2定义一个字符串变量sImage_describer，用于存储图像描述符的文件路径。使用stlplus::create_filespec函数创建文件路径，
    该函数将目录directory_match和文件名image_describer组合在一起。

    3使用std::unique_ptr<Regions>定义一个智能指针regions_type，用于存储图像描述符的类型。

    4使用Init_region_type_from_file函数从文件sImage_describer中读取图像描述符的类型，并将其转换为Regions类型的智能指针。该函数的实现可能如下所示：
    std::unique_ptr<Regions> Init_region_type_from_file(const std::string& sImage_describer)
    {   
        // 读取文件内容
        std::ifstream file(sImage_describer);
        if (!file.is_open())
        {
            return nullptr;
        }
    
        // 解析文件内容，创建Regions类型的智能指针
        std::unique_ptr<Regions> regions_type = std::make_unique<Regions>();
        regions_type->read(file);
    
        return regions_type;
    }
    5如果regions_type为空，则输出错误信息并返回失败状态。
    总之，这段代码的作用是从指定的文件中读取图像描述符的类型，并将其转换为Regions类型的智能指针。如果文件读取失败，将返回失败状态。

  */
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(directory_match, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
    return EXIT_FAILURE;
  }

  // Features reading

  /*
    这段C++代码是用于加载3D场景重建所需的特征（features）数据。std::shared_ptr<Features_Provider>是一个智能指针，
    用于管理Features_Provider类的实例。std::make_shared<Features_Provider>()创建了一个新的Features_Provider对象，
    并将其转换为智能指针。load()函数用于加载特征数据，它接受三个参数：sfm_data（3D场景数据）、directory_match（特征数据的目录）
    和regions_type（特征类型）。如果加载失败，load()函数将返回false，程序将输出错误信息并退出。
  */
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, directory_match, regions_type)) {
    OPENMVG_LOG_ERROR << "Cannot load view corresponding features in directory: " << directory_match << ".";
    return EXIT_FAILURE;
  }
  // Matches reading

  /*
    这段C++代码主要实现了以下功能：

    创建一个Matches_Provider类的共享指针，用于加载匹配文件。
    尝试读取提供的匹配文件名或默认的匹配文件（matches.f.txt/bin）。
    根据scene_initializer_enum的值选择不同的场景初始化方法。
    创建一个SfMSceneInitializer类的unique指针，用于初始化场景。
    根据sfm_engine_type的值选择不同的重建引擎。
    这段代码主要用于OpenMVG（一个开源的计算机视觉库）中的场景重建功能。在使用OpenMVG进行场景重建时，
    需要先准备好图像和特征点匹配信息，然后通过这段代码进行场景的初始化和重建。

    注意事项：

    确保已经正确安装了OpenMVG库，并正确配置了环境变量。
    确保图像和特征点匹配信息已经准备好，并且格式正确。
    根据实际情况选择合适的场景初始化方法和重建引擎。
    注意代码中的错误处理和返回值，确保程序在出现错误时能够正常退出。
  */
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if // Try to read the provided match filename or the default one (matches.f.txt/bin)
  (
  !(matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, filename_match)) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.f.txt")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.f.bin")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.e.txt")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.e.bin")))
      )
  {
    OPENMVG_LOG_ERROR << "Cannot load the match file.";
    return EXIT_FAILURE;
  }
/*
    这段代码是用于根据不同的枚举值来创建一个std::unique_ptr<SfMSceneInitializer>类型的对象scene_initializer。
    SfMSceneInitializer是一个用于初始化SfM场景的类，它有多种方法，如自动配对、最大配对、现有姿态和恒星初始化。
*/
  std::unique_ptr<SfMSceneInitializer> scene_initializer;
  switch(scene_initializer_enum)
  {
  case ESfMSceneInitializer::INITIALIZE_AUTO_PAIR:
    OPENMVG_LOG_ERROR << "Not yet implemented.";
    return EXIT_FAILURE;
    break;
  case ESfMSceneInitializer::INITIALIZE_MAX_PAIR:
    scene_initializer.reset(new SfMSceneInitializerMaxPair(sfm_data,
                                 feats_provider.get(),
                                 matches_provider.get()));
    break;
  case ESfMSceneInitializer::INITIALIZE_EXISTING_POSES:
    scene_initializer.reset(new SfMSceneInitializer(sfm_data,
                            feats_provider.get(),
                            matches_provider.get()));
    break;
  case ESfMSceneInitializer::INITIALIZE_STELLAR:
    scene_initializer.reset(new SfMSceneInitializerStellar(sfm_data,
                                 feats_provider.get(),
                                 matches_provider.get()));
    break;
  default:
    OPENMVG_LOG_ERROR << "Unknown SFM Scene initializer method";
    return EXIT_FAILURE;
  }
  if (!scene_initializer)
  {
    OPENMVG_LOG_ERROR << "Invalid scene initializer.";
    return EXIT_FAILURE;
  }


  std::unique_ptr<ReconstructionEngine> sfm_engine;
  switch (sfm_engine_type)
  {
  case ESfMEngine::INCREMENTAL:
  {
    /*
    这段C++代码是用于创建一个SequentialSfMReconstructionEngine对象，并配置其参数。SequentialSfMReconstructionEngine是一个
    用于进行序列式SfM（单目视觉SLAM）重建的类。

    首先，创建一个SequentialSfMReconstructionEngine对象，传入三个参数：sfm_data（SfM数据）、directory_output（输出目录）
    和 Reconstruction_Report.html（重建报告的HTML文件路径）。

    配置FeaturesProvider和MatchesProvider，这两个对象分别用于提供特征点和匹配关系。

    配置相机模型类型（EINTRINSIC(user_camera_model)），可以是针孔相机模型、鱼眼相机模型等。

    配置三角测量方法（ETriangulationMethod(triangulation_method)），可以是基本三角测量方法、RANSAC三角测量方法等。

    配置重投影方法（resection::SolverType(resection_method)），可以是基本重投影方法、RANSAC重投影方法等。

    总之，这段代码的作用是创建一个SequentialSfMReconstructionEngine对象，并为其配置相应的参数，以便进行序列式SfM重建。
    */
    SequentialSfMReconstructionEngine * engine =
        new SequentialSfMReconstructionEngine(
          sfm_data,
          directory_output,
          stlplus::create_filespec(directory_output, "Reconstruction_Report.html"));

    // Configuration:
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
    engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
    engine->SetResectionMethod(static_cast<resection::SolverType>(resection_method));
/*
    这段C++代码是OpenMVG库中的一个部分，主要用于处理初始对（initial pair）参数。
    OpenMVG是一个开源的计算机视觉库，用于从图像中恢复场景的结构（3D几何结构）和 appearance（外观）。

    具体来说，这段代码的主要作用是：

    检查初始对是否已经设置。如果已经设置，那么就计算初始对在SfM数据（sfm_data）中的索引（initial_pair_index）。
    如果计算成功，将初始对索引设置给SfM引擎（engine）。
    重置SfM引擎（sfm_engine）。
    这段代码的注意事项：

    确保已经正确设置了初始对（initial_pair_string.first和initial_pair_string.second）。如果未设置，或者设置错误，那么会输出错误信息并返回失败状态。
    确保已经正确初始化了SfM数据（sfm_data）和SfM引擎（engine）。
*/
    // Handle Initial pair parameter
    if (!initial_pair_string.first.empty() && !initial_pair_string.second.empty())
    {
      Pair initial_pair_index;
      if (!computeIndexFromImageNames(sfm_data, initial_pair_string, initial_pair_index))
      {
        OPENMVG_LOG_ERROR << "Could not find the initial pairs <" << initial_pair_string.first
                  <<  ", " << initial_pair_string.second << ">!";
        return EXIT_FAILURE;
      }
      engine->setInitialPair(initial_pair_index);
    }

    sfm_engine.reset(engine);
  }
    break;
    /*
        这段代码是C++中一个switch语句的其中一个case，用于创建一个名为SequentialSfMReconstructionEngine2的类对象。
        这个类对象是用于进行增量式三维重建的。

        具体来说，这段代码的实现原理如下：

        首先，通过new关键字创建一个SequentialSfMReconstructionEngine2类的对象，该对象被命名为engine。

        然后，将scene_initializer、sfm_data、directory_output和stlplus::create_filespec(directory_output, "Reconstruction_Report.html")
        等参数传递给engine对象的构造函数。这些参数分别用于初始化场景、存储三维重建结果的数据结构、输出目录和重建报告的文件名。

        接着，将feats_provider和matches_provider这两个对象传递给engine对象的SetFeaturesProvider和SetMatchesProvider方法，
        用于提供特征点和匹配关系。

        然后，通过SetTriangulationMethod、SetUnknownCameraType和SetResectionMethod方法，配置重建参数，
        包括三角测量方法、未知相机模型和重投影方法。
        
        最后，将engine对象赋值给sfm_engine，以便后续使用。

        需要注意的是，这段代码中的ESfMEngine::INCREMENTALV2是一个枚举类型，
        用于表示增量式三维重建的版本。在实际应用中，需要根据具体情况进行选择。
    */
  case ESfMEngine::INCREMENTALV2:
  {
    SequentialSfMReconstructionEngine2 * engine =
        new SequentialSfMReconstructionEngine2(
          scene_initializer.get(),
          sfm_data,
          directory_output,
          stlplus::create_filespec(directory_output, "Reconstruction_Report.html"));

    // Configuration:
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
    engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
    engine->SetResectionMethod(static_cast<resection::SolverType>(resection_method));

    sfm_engine.reset(engine);
  }
    break;
    /*
        这段代码是C++中一个switch语句的其中一个case，用于创建一个全局SfM（三维重建）引擎。
        具体来说，它创建了一个名为GlobalSfMReconstructionEngine_RelativeMotions的全局SfM引擎，该引擎用于从相对运动中恢复三维场景。

        首先，代码创建了一个指向GlobalSfMReconstructionEngine_RelativeMotions类的指针engine，并使用new关键字分配内存。
        然后，它将sfm_data（三维重建数据）、directory_output（输出目录）和Reconstruction_Report.html（重建报告文件名）作为参数传递给引擎的构造函数。
        接下来，代码配置了引擎的一些属性，例如特征提供者和匹配提供者。最后，它将engine指针赋值给sfm_engine，并使用reset方法释放内存。

        需要注意的是，这段代码只是一个示例，具体实现可能因应用场景而异。在实际应用中，您可能需要根据具体需求进行调整。

    */
  case ESfMEngine::GLOBAL:
  {
    GlobalSfMReconstructionEngine_RelativeMotions * engine =
        new GlobalSfMReconstructionEngine_RelativeMotions(
          sfm_data,
          directory_output,
          stlplus::create_filespec(directory_output, "Reconstruction_Report.html"));

    // Configuration:
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure motion averaging method
    engine->SetRotationAveragingMethod(ERotationAveragingMethod(rotation_averaging_method));
    engine->SetTranslationAveragingMethod(ETranslationAveragingMethod(translation_averaging_method));

    sfm_engine.reset(engine);
  }
  break;
  /*
    这段代码是C++中一个switch语句的case分支，用于创建一个名为StellarSfMReconstructionEngine的类实例。
    这个类实例用于执行基于StellarSfM（一种基于视觉SLAM的SfM方法）的3D重建。

    1.首先，创建一个新的StellarSfMReconstructionEngine对象，并将其设置为当前的SfM引擎。
    2.然后，将sfm_data（一个包含所有图像和相机参数的数据结构）和directory_output（输出目录）传递给StellarSfMReconstructionEngine对象。
    3.接着，创建一个HTML报告文件，用于记录重建过程中的信息。
    4.设置特征提供者和匹配提供者，这两个对象分别用于提供图像特征点和匹配关系。
    5.配置重建参数，包括图形简化方法（graph_simplification_method）和图形简化值（graph_simplification_value）。
    6.将StellarSfMReconstructionEngine对象设置为当前的SfM引擎。

    注意：这段代码主要用于创建一个StellarSfMReconstructionEngine对象，并配置其参数。在实际应用中，可能需要根据具体情况进行调整。
  */
  case ESfMEngine::STELLAR:
  {
    StellarSfMReconstructionEngine * engine =
      new StellarSfMReconstructionEngine(
        sfm_data,
        directory_output,
        stlplus::create_filespec(directory_output, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    engine->SetGraphSimplification(graph_simplification_method, graph_simplification_value);

    sfm_engine.reset(engine);
  }
  break;
  default:
  break;
  }
  /*
    这段C++代码是用于设置SfM（单目视觉SLAM）引擎的内部参数和外部参数，以及是否使用运动先验。

    1.首先，检查sfm_engine是否为空，如果为空，则输出错误信息并返回失败。
    2.如果sfm_engine不为空，则设置内参和外参的 refinement 类型。
    这里使用了两个枚举值intrinsic_refinement_options和extrinsic_refinement_options来表示内参和外参的 refinement 类型。
    3.设置是否使用运动先验。b_use_motion_priors是一个布尔值，表示是否使用运动先验。

    总之，这段代码的主要作用是设置SfM引擎的内部参数和外部参数，以及是否使用运动先验。
  */
  if (!sfm_engine)
  {
    OPENMVG_LOG_ERROR << "Cannot create the requested SfM Engine.";
    return EXIT_FAILURE;
  }

  sfm_engine->Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
  sfm_engine->Set_Extrinsics_Refinement_Type(extrinsic_refinement_options);
  sfm_engine->Set_Use_Motion_Prior(b_use_motion_priors);

  //---------------------------------------
  // Sequential reconstruction process
  //---------------------------------------
/*
    这段C++代码是OpenMVG库中的一个片段，主要用于执行三维重建过程并保存结果。以下是代码的详细解释：

    首先，创建一个openMVG::system::Timer对象timer，用于计算整个三维重建过程所需的时间。

    使用sfm_engine->Process()开始执行三维重建过程。如果重建成功，执行以下操作：

    a. 输出整个三维重建过程所需的时间。

    b. 生成一个名为SfM_Report.html的HTML报告，用于展示重建结果。

    c. 将计算得到的三维场景数据和可查看的结果导出到磁盘。这里分别导出了一个名为sfm_data.bin的二进制文件和一个名为cloud_and_poses.ply的PLY文件。

    如果重建失败，返回EXIT_FAILURE。

    总之，这段代码的主要目的是执行三维重建并保存结果。在执行过程中，会计算重建所需的时间，并生成一个HTML报告展示重建结果。
    最后，将重建得到的数据和结果导出到磁盘。
*/
  openMVG::system::Timer timer;

  if (sfm_engine->Process())
  {
    OPENMVG_LOG_INFO << " Total Sfm took (s): " << timer.elapsed();

    OPENMVG_LOG_INFO << "...Generating SfM_Report.html";
    Generate_SfM_Report(sfm_engine->Get_SfM_Data(),
              stlplus::create_filespec(directory_output, "SfMReconstruction_Report.html"));

    //-- Export to disk computed scene (data & viewable results)
    OPENMVG_LOG_INFO << "...Export SfM_Data to disk.";
    Save(sfm_engine->Get_SfM_Data(),
       stlplus::create_filespec(directory_output, "sfm_data", ".bin"),
       ESfM_Data(ALL));

    Save(sfm_engine->Get_SfM_Data(),
       stlplus::create_filespec(directory_output, "cloud_and_poses", ".ply"),
       ESfM_Data(ALL));

    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}


/*

def main(argc, argv):
    if argc < 2:
        print("Usage: %s <config_file>" % argv[0])
        return 1

    # Load the configuration
    config = Configuration(argv[1])

    # Retrieve the parameters
    rotation_averaging_method = config.rotation_averaging_method()
    translation_averaging_method = config.translation_averaging_method()
    graph_simplification = config.graph_simplification()
    graph_simplification_value = config.graph_simplification_value()
    user_camera_model = config.user_camera_model()
    triangulation_method = config.triangulation_method()
    resection_method = config.resection_method()
    intrinsic_refinement_options = config.intrinsic_refinement_options()
    extrinsic_refinement_options = config.extrinsic_refinement_options()
    b_use_motion_priors = config.use_motion_priors()
    filename_sfm_data = config.input_sfm_data_filename()
    filename_match = config.input_match_filename()
    directory_match = config.match_directory()
    directory_output = config.output_directory()

    # Check the parameters
    if not os.path.isfile(filename_sfm_data):
        print("The input SfM_Data file does not exist.")
        return 1

    if not os.path.isfile(filename_match):
        print("The input match file does not exist.")
        return 1

    if not os.path.isdir(directory_output):
        print("The output directory does not exist.")
        return 1

    # Init the regions_type from the image describer file (used for image regions extraction)
    regions_type = None
    sImage_describer = stlplus::create_filespec(directory_match, "image_describer", "json")
    if os.path.isfile(sImage_describer):
        regions_type = Init_region_type_from_file(sImage_describer)

    if regions_type is None:
        print("Invalid: ", sImage_describer, " regions type file.")
        return 1

    # Features reading
    feats_provider = Features_Provider()
    if not feats_provider.load(sfm_data, directory_match):
        print("Cannot load view corresponding features in directory: ", directory_match)
        return 1

    # Matches reading
    matches_provider = Matches_Provider()
    if not matches_provider.load(sfm_data, stlplus::create_filespec(directory_match, filename_match)):
        print("Cannot load the match file.")
        return 1

    # Handle Initial pair parameter
    initial_pair = config.initial_pair()
    if initial_pair.first != "" and initial_pair.second != "":
        if not computeIndexFromImageNames(sfm_data, initial_pair, initial_pair_index):
            print("Could not find the initial pairs <", initial_pair.first, ", ", initial_pair.second, ">!")
            return 1

    # Create the SfM engine
    sfm_engine = None
    engine_name = config.reconstruction_engine_type()
    if engine_name == "INCREMENTAL":
        sfm_engine = SequentialSfMReconstructionEngine(sfm_data, directory_output, stlplus::create_filespec(directory_output, "Reconstruction_Report.html"))
    elif engine_name == "INCREMENTALV2":
        sfm_engine = SequentialSfMReconstructionEngine2(sfm_data, directory_output, stlplus::create_filespec(directory_output, "Reconstruction_Report.html"))
    elif engine_name == "GLOBAL":
        sfm_engine = GlobalSfMReconstructionEngine_RelativeMotions(sfm_data, directory_output, stlplus::create_filespec(directory_output, "Reconstruction_Report





*/


/*

def main(argc, char **argv):
    # Init command line parameters
    # ...

    # Load input SfM_Data scene
    # ...

    # Init the regions_type from the image describer file (used for image regions extraction)
    # ...

    # Features reading
    # ...

    # Matches reading
    # ...

    # Handle Initial pair parameter
    # ...

    # Create the SfM engine
    # ...

    # Set the Intrinsics & Extrinsics refinement types
    # ...

    # Set the Use_Motion_Prior flag
    # ...

    # Sequential reconstruction process
    # ...

if __name__ == "__main__":
    main(len(sys.argv), sys.argv)
    */