
// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2019 Pierre MOULON, Romuald PERROT

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_stats.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"
#include "openMVG/matching_image_collection/Eo_Robust.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <locale>
#include <memory>
#include <string>

//python Embed
#include<pybind11/pybind11.h>
#include<pybind11/embed.h>

namespace py = pybind11;
using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;

enum EGeometricModel
{
    FUNDAMENTAL_MATRIX = 0,
    ESSENTIAL_MATRIX = 1,
    HOMOGRAPHY_MATRIX = 2,
    ESSENTIAL_MATRIX_ANGULAR = 3,
    ESSENTIAL_MATRIX_ORTHO = 4,
    ESSENTIAL_MATRIX_UPRIGHT = 5
};

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
/// - Compute geometric coherent feature matches (robust model estimation from putative matches)
/// - Export computed data
int main(int argc, char** argv)
{
    py::scoped_interpreter guard{};

    auto main_GeometricFilter_Module = py::module_::import("main_GeometricFilter");
 auto print_usage = main_GeometricFilter_Module.attr("print_usage");
 auto print_usage_info = main_GeometricFilter_Module.attr("print_usage_info");
 auto get_geometric_model = main_GeometricFilter_Module.attr("get_geometric_model");
 auto create_and_export_graph = main_GeometricFilter_Module.attr("create_and_export_graph");
    CmdLine cmd;

    // The scene
    std::string sSfM_Data_Filename;
    // The input matches
    std::string sPutativeMatchesFilename;
    // The output matches
    std::string sFilteredMatchesFilename;
    // The input pairs
    std::string sInputPairsFilename;
    // The output pairs
    std::string sOutputPairsFilename;

    std::string  sGeometricModel = "f";
    bool         bForce = false;
    bool         bGuided_matching = false;
    int          imax_iteration = 2048;
    unsigned int ui_max_cache_size = 0;

    //required
    cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
    cmd.add(make_option('o', sFilteredMatchesFilename, "output_file"));
    cmd.add(make_option('m', sPutativeMatchesFilename, "matches"));
    // Options
    cmd.add(make_option('p', sInputPairsFilename, "input_pairs"));
    cmd.add(make_option('s', sOutputPairsFilename, "output_pairs"));
    cmd.add(make_option('g', sGeometricModel, "geometric_model"));
    cmd.add(make_option('f', bForce, "force"));
    cmd.add(make_option('r', bGuided_matching, "guided_matching"));
    cmd.add(make_option('I', imax_iteration, "max_iteration"));
    cmd.add(make_option('c', ui_max_cache_size, "cache_size"));

    try
    {
        if (argc == 1)
            throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    }
    catch (const std::string& s)
    {
        std::string usage = "Usage: " + std::string(argv[0]) + "\n";
        print_usage(usage);
        OPENMVG_LOG_INFO << s;
        return EXIT_FAILURE;
    }

    std::string program_name = argv[0];
    std::string input_file = "example_input.txt";
    std::string matches_file = "example_matches.txt";
    std::string output_file = "example_output.txt";
    std::string input_pairs = "example_pairs.txt";
    std::string output_pairs = "example_filtered_pairs.txt";
    bool force = false;
    std::string geometric_model = "f";
    bool guided_matching = false;
    std::string cache_size = "unlimited"

    py::str py_program_name = py::cast(program_name);
    py::str py_input_file = py::cast(input_file);
    py::str py_matches_file = py::cast(matches_file);
    py::str py_output_file = py::cast(output_file);
    py::str py_input_pairs = py::cast(input_pairs);
    py::str py_output_pairs = py::cast(output_pairs);
    py::str py_geometric_model = py::cast(geometric_model);
    py::str py_cache_size = py::cast(cache_size);
    print_usage_info(py_program_name, py_input_file, py_matches_file, py_output_file,
        py_input_pairs, py_output_pairs, force, py_geometric_model,
        guided_matching, py_cache_size);

    if (sFilteredMatchesFilename.empty())
    {
        OPENMVG_LOG_ERROR << "It is an invalid output file";
        return EXIT_FAILURE;
    }
    if (sSfM_Data_Filename.empty())
    {
        OPENMVG_LOG_ERROR << "It is an invalid SfM file";
        return EXIT_FAILURE;
    }
    if (sPutativeMatchesFilename.empty())
    {
        OPENMVG_LOG_ERROR << "It is an invalid putative matche file";
        return EXIT_FAILURE;
    }

    const std::string sMatchesDirectory = stlplus::folder_part(sPutativeMatchesFilename);

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    enum GeometricModel {
        FUNDAMENTAL_MATRIX,
        ESSENTIAL_MATRIX,
        HOMOGRAPHY_MATRIX,
        ESSENTIAL_MATRIX_ANGULAR,
        ESSENTIAL_MATRIX_UPRIGHT,
        ESSENTIAL_MATRIX_ORTHO
    };
    char model_char = std::tolower(sGeometricModel[0]);

    try {
        
        GeometricModel eGeometricModelToCompute = py::cast<GeometricModel>(get_geometric_model(model_char));

        std::cout << "Selected geometric model: " << eGeometricModelToCompute << std::endl;
    }
    catch (const py::cast_error& e) {
     
        OPENMVG_LOG_ERROR << "Failed to cast Python return value: " << e.what();
        return EXIT_FAILURE;
    }
    catch (const py::error_already_set& e) {
      
        OPENMVG_LOG_ERROR << "Python exception occurred: " << py::print_exception();
        return EXIT_FAILURE;
    }

    // -----------------------------
    // - Load SfM_Data Views & intrinsics data
    // a. Load putative descriptor matches
    // [a.1] Filter matches with input pairs
    // b. Geometric filtering of putative matches
    // + Export some statistics
    // -----------------------------

    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS)))
    {
        OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read.";
        return EXIT_FAILURE;
    }

    //---------------------------------------
    // Load SfM Scene regions
    //---------------------------------------
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    // Consider that the image_describer.json is inside the matches directory (which is bellow the sfm_data.bin)
    const std::string        sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer.json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
        return EXIT_FAILURE;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //    - Descriptor matching (according user method choice)
    //    - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------

    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider;
    if (ui_max_cache_size == 0)
    {
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    }
    else
    {
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }

    // Show the progress on the command line:
    system::LoggerProgress progress;

    if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress))
    {
        OPENMVG_LOG_ERROR << "Invalid regions.";
        return EXIT_FAILURE;
    }

    PairWiseMatches map_PutativeMatches;
    //---------------------------------------
    // A. Load initial matches
    //---------------------------------------
    if (!Load(map_PutativeMatches, sPutativeMatchesFilename))
    {
        OPENMVG_LOG_ERROR << "Failed to load the initial matches file.";
        return EXIT_FAILURE;
    }

    if (!sInputPairsFilename.empty())
    {
        // Load input pairs
        OPENMVG_LOG_INFO << "Loading input pairs ...";
        Pair_Set input_pairs;
        loadPairs(sfm_data.GetViews().size(), sInputPairsFilename, input_pairs);

        // Filter matches with the given pairs
        OPENMVG_LOG_INFO << "Filtering matches with the given pairs.";
        map_PutativeMatches = getPairs(map_PutativeMatches, input_pairs);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //    - AContrario Estimation of the desired geometric model
    //    - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------

    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
        new ImageCollectionGeometricFilter(&sfm_data, regions_provider));

    if (filter_ptr)
    {
        system::Timer timer;
        const double  d_distance_ratio = 0.6;

        PairWiseMatches map_GeometricMatches;
        switch (eGeometricModelToCompute)
        {
        case HOMOGRAPHY_MATRIX:
        {
            const bool bGeometric_only_guided_matching = true;
            filter_ptr->Robust_model_estimation(
                GeometricFilter_HMatrix_AC(4.0, imax_iteration),
                map_PutativeMatches,
                bGuided_matching,
                bGeometric_only_guided_matching ? -1.0 : d_distance_ratio,
                &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case FUNDAMENTAL_MATRIX:
        {
            filter_ptr->Robust_model_estimation(
                GeometricFilter_FMatrix_AC(4.0, imax_iteration),
                map_PutativeMatches,
                bGuided_matching,
                d_distance_ratio,
                &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX:
        {
            filter_ptr->Robust_model_estimation(
                GeometricFilter_EMatrix_AC(4.0, imax_iteration),
                map_PutativeMatches,
                bGuided_matching,
                d_distance_ratio,
                &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();

            //-- Perform an additional check to remove pairs with poor overlap
            std::vector<PairWiseMatches::key_type> vec_toRemove;
            for (const auto& pairwisematches_it : map_GeometricMatches)
            {
                const size_t putativePhotometricCount = map_PutativeMatches.find(pairwisematches_it.first)->second.size();
                const size_t putativeGeometricCount = pairwisematches_it.second.size();
                const float  ratio = putativeGeometricCount / static_cast<float>(putativePhotometricCount);
                if (putativeGeometricCount < 50 || ratio < .3f)
                {
                    // the pair will be removed
                    vec_toRemove.push_back(pairwisematches_it.first);
                }
            }
            //-- remove discarded pairs
            for (const auto& pair_to_remove_it : vec_toRemove)
            {
                map_GeometricMatches.erase(pair_to_remove_it);
            }
        }
        break;
        case ESSENTIAL_MATRIX_ANGULAR:
        {
            filter_ptr->Robust_model_estimation(
                GeometricFilter_ESphericalMatrix_AC_Angular<false>(4.0, imax_iteration),
                map_PutativeMatches, bGuided_matching, d_distance_ratio, &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX_UPRIGHT:
        {
            filter_ptr->Robust_model_estimation(
                GeometricFilter_ESphericalMatrix_AC_Angular<true>(4.0, imax_iteration),
                map_PutativeMatches, bGuided_matching, d_distance_ratio, &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX_ORTHO:
        {
            filter_ptr->Robust_model_estimation(
                GeometricFilter_EOMatrix_RA(2.0, imax_iteration),
                map_PutativeMatches,
                bGuided_matching,
                d_distance_ratio,
                &progress);
            map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        }

        //---------------------------------------
        //-- Export geometric filtered matches
        //---------------------------------------
        if (!Save(map_GeometricMatches, sFilteredMatchesFilename))
        {
            OPENMVG_LOG_ERROR << "Cannot save filtered matches in: " << sFilteredMatchesFilename;
            return EXIT_FAILURE;
        }

        // -- export Geometric View Graph statistics
        graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_GeometricMatches));

        OPENMVG_LOG_INFO << "Task done in (s): " << timer.elapsed();

        //-- export Adjacency matrix
        OPENMVG_LOG_INFO << "\n Export Adjacency Matrix of the pairwise's geometric matches";

        PairWiseMatchingToAdjacencyMatrixSVG(sfm_data.GetViews().size(),
            map_GeometricMatches,
            stlplus::create_filespec(sMatchesDirectory, "GeometricAdjacencyMatrix", "svg"));

        const Pair_Set outputPairs = getPairs(map_GeometricMatches);

        //-- export view pair graph once geometric filter have been done
        {
            std::set<IndexT> set_ViewIds;
            std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(), std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(set_ViewIds, outputPairs);
            graph::exportToGraphvizData(
                stlplus::create_filespec(sMatchesDirectory, "geometric_matches"),
                putativeGraph);
        }
        using IndexT = int; 
        using ViewIds = std::vector<IndexT>;
        const char* python_module_name = "geometric_graph";
        const char* python_function_name = "create_and_export_graph";
        if (!py_module.def(python_function_name)) {
            std::cerr << "Error: Unable to find function " << python_function_name << " in module " << python_module_name << std::endl;
            return 1;
        }
        ViewIds view_ids = { 1, 2, 3, 4, 5 }; 
        std::vector<std::pair<IndexT, IndexT>> outputPairs = { {1, 2}, {2, 3}, {3, 4} };
        std::string sMatchesDirectory = "/path/to/matches/directory"; 
        py::object py_result = py_module.attr(python_function_name)(view_ids, outputPairs, sMatchesDirectory);

        // Write pairs
        if (!sOutputPairsFilename.empty())
        {
            OPENMVG_LOG_INFO << "Saving pairs to: " << sOutputPairsFilename;
            if (!savePairs(sOutputPairsFilename, outputPairs))
            {
                OPENMVG_LOG_ERROR << "Failed to write pairs file";
                return EXIT_FAILURE;
            }
        }
    }
    return EXIT_SUCCESS;
}
