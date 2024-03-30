from enum import Enum


#77
def OPENMVG_LOG_INFO_Usage():
    print( "[-i|--input_file]   A SfM_Data file\n"
            "[-o|--output_file]  Output file where computed matches are stored\n"
            "[-p|--pair_list]    Pairs list file\n"
            "[-f|--force] Force to recompute data]\n"
            "[-r|--ratio] Distance ratio to discard non meaningful matches\n"
            "   0.8: (default).\n"
            "[-n|--nearest_matching_method]\n"
            "  AUTO: auto choice from regions type,\n"
            "  For Scalar based regions descriptor:\n"
            "    BRUTEFORCEL2: L2 BruteForce matching,\n"
            "    HNSWL2: L2 Approximate Matching with Hierarchical Navigable Small World graphs,\n"
            "    HNSWL1: L1 Approximate Matching with Hierarchical Navigable Small World graphs\n"
            "      tailored for quantized and histogram based descriptors (e.g uint8 RootSIFT)\n"
            "    ANNL2: L2 Approximate Nearest Neighbor matching,\n"
            "    CASCADEHASHINGL2: L2 Cascade Hashing matching.\n"
            "    FASTCASCADEHASHINGL2: (default)\n"
            "      L2 Cascade Hashing with precomputed hashed regions\n"
            "     (faster than CASCADEHASHINGL2 but use more memory).\n"
            "  For Binary based descriptor:\n"
            "    BRUTEFORCEHAMMING: BruteForce Hamming matching,\n"
            "    HNSWHAMMING: Hamming Approximate Matching with Hierarchical Navigable Small World graphs\n"
            "[-c|--cache_size]\n"
            "  Use a regions cache (only cache_size regions will be stored in memory)\n"
            "  If not used, all regions will be load in memory."
            "\n[Pre-emptive matching:]\n"
            "[-P|--preemptive_feature_count] <NUMBER> Number of feature used for pre-emptive matching")

#113
def OPENMVG_LOG_INFO_OUTPUT(argv,sSfM_Data_Filename,sOutputMatchesFilename,sPredefinedPairList,bForce,fDistRatio,sNearestMatchingMethod,ui_max_cache_size,cmdusedP,ui_preemptive_feature_count):
    cache_size_str=""
    if ui_max_cache_size==0:
        cache_size_str="unlimited"  
    else:
        cache_size_str=ui_max_cache_size

    print(  "You called :\n",
             str(argv) + "\n",
             "--input_file "+ str(sSfM_Data_Filename)+ "\n",
             "--output_file "+ str(sOutputMatchesFilename)+ "\n",
             "--pair_list "+ str(sPredefinedPairList)+ "\n",
             "Optional parameters:",
             "\n",
             "--force "+ str(bForce)+ "\n",
             "--ratio "+ str(fDistRatio)+ "\n",
             "--nearest_matching_method "+ str(sNearestMatchingMethod)+ "\n",
             "--cache_size "+ str(cache_size_str) + "\n",
             "--preemptive_feature_used/count "+ str(cmdusedP)+ " / "+ str(ui_preemptive_feature_count))

#233
class EMatcherType(Enum):
    BRUTE_FORCE_L2 = 0
    ANN_L2=1
    CASCADE_HASHING_L2=2
    HNSW_L2=3
    HNSW_L1=4
    BRUTE_FORCE_HAMMING=5
    HNSW_HAMMING=6
    AUTO=7#special
    FASTCASCADEHASHINGL2=8#special
    NONE=9#special

def sNearestMatchingMethod_Divide(sNearestMatchingMethod):
    if sNearestMatchingMethod=="AUTO":return EMatcherType.AUTO
    if sNearestMatchingMethod == "BRUTEFORCEL2":return EMatcherType.BRUTE_FORCE_L2
    if sNearestMatchingMethod == "BRUTEFORCEHAMMING":return EMatcherType.BRUTE_FORCE_HAMMING
    if sNearestMatchingMethod == "HNSWL2":return EMatcherType.HNSW_L2
    if sNearestMatchingMethod == "HNSWL1":return EMatcherType.HNSW_L1
    if sNearestMatchingMethod == "HNSWHAMMING":return EMatcherType.HNSW_HAMMING 
    if sNearestMatchingMethod == "ANNL2":return EMatcherType.ANN_L2
    if sNearestMatchingMethod == "CASCADEHASHINGL2":return EMatcherType.CASCADE_HASHING_L2
    if sNearestMatchingMethod == "FASTCASCADEHASHINGL2":return EMatcherType.FASTCASCADEHASHINGL2
    return EMatcherType.NONE


if __name__ == "__main__":
    print("test","s")
    OPENMVG_LOG_INFO_Usage()
    OPENMVG_LOG_INFO_OUTPUT(0,0,0,0,0,0,0,0,0,0)