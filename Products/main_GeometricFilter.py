def print_usage():
    print("Usage: " , argv[0] , '\n'
       , "[-i|--input_file]       A SfM_Data file\n"
       , "[-m|--matches]          (Input) matches filename\n"
       , "[-o|--output_file]      (Output) filtered matches filename\n"
       , "\n[Optional]\n"
       , "[-p|--input_pairs]      (Input) pairs filename\n"
       , "[-s|--output_pairs]     (Output) filtered pairs filename\n"
       , "[-f|--force]            Force to recompute data\n"
       , "[-g|--geometric_model]\n"
       , "  (pairwise correspondences filtering thanks to robust model estimation):\n"
       , "   f: (default) fundamental matrix,\n"
       , "   e: essential matrix,\n"
       , "   h: homography matrix.\n"
       , "   a: essential matrix with an angular parametrization,\n"
       , "   u: upright essential matrix with an angular parametrization,\n"
       , "   o: orthographic essential matrix.\n"
       , "[-r|--guided_matching]  Use the found model to improve the pairwise correspondences.\n"
       , "[-c|--cache_size]\n"
       , "  Use a regions cache (only cache_size regions will be stored in memory)\n"
       , "  If not used, all regions will be load in memory.")
def print_usage_info(program_name, input_file, matches_file, output_file, input_pairs, output_pairs, force, geometric_model, guided_matching, cache_size):
    print(f" You called : ")
    print(f"\n{program_name}")
    print(f"--input_file:           {input_file}")
    print(f"--matches:              {matches_file}")
    print(f"--output_file:          {output_file}")
    print("Optional parameters: ")
    print(f"--input_pairs           {input_pairs}")
    print(f"--output_pairs          {output_pairs}")
    print(f"--force                 {force}")
    print(f"--geometric_model       {geometric_model}")
    print(f"--guided_matching       {guided_matching}")
    print(f"--cache_size            {cache_size}")
def get_geometric_model(model_char):
    if model_char.lower() == 'f':
        return FUNDAMENTAL_MATRIX
    elif model_char.lower() == 'e':
        return ESSENTIAL_MATRIX
    elif model_char.lower() == 'h':
        return HOMOGRAPHY_MATRIX
    elif model_char.lower() == 'a':
        return ESSENTIAL_MATRIX_ANGULAR
    elif model_char.lower() == 'u':
        return ESSENTIAL_MATRIX_UPRIGHT
    elif model_char.lower() == 'o':
        return ESSENTIAL_MATRIX_ORTHO
    else:
        raise ValueError("Unknown geometric model")
def create_and_export_graph(view_ids, output_pairs, matches_directory):
 
    putative_graph = create_graph(view_ids, output_pairs)
    
    graphviz_data = export_to_graphviz(putative_graph)
    
    graphviz_file = f"{matches_directory}/geometric_matches.gv"
    with open(graphviz_file, 'w') as f:
        f.write(graphviz_data)