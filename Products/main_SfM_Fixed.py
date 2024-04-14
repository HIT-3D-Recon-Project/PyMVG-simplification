from enum import Enum

def test():
	print("test")

def Init_Show():
    print("\n-----------------------------------------------------------"
          "\n Structure from Motion:"
          "\n-----------------------------------------------------------")


def Information_Show(argv,graph_simplification_value,
                    triangulation_method,DIRECT_LINEAR_TRANSFORM,L1_ANGULAR,LINFINITY_ANGULAR,INVERSE_DEPTH_WEIGHTED_MIDPOINT,
                    resection_method,DLT_6POINTS,P3P_KE_CVPR17,P3P_KNEIP_CVPR11,P3P_NORDBERG_ECCV18,UP2P_KUKELOVA_ACCV10):
	print("Usage: " , argv , '\n'
      , "[Required]\n"
      , "[-i|--input_file] path to a SfM_Data scene\n"
      , "[-m|--match_dir] path to the matches that corresponds to the provided SfM_Data scene\n"
      , "[-o|--output_dir] path where the output data will be stored\n"
      , "[-s|--sfm_engine] Type of SfM Engine to use for the reconstruction\n"
      , "\t INCREMENTAL   : add image sequentially to a 2 view seed\n"
      , "\t INCREMENTALV2 : add image sequentially to a 2 or N view seed (experimental)\n"
      , "\t GLOBAL        : initialize globally rotation and translations\n"
      , "\t STELLAR       : n-uplets local motion refinements + global SfM\n"
      , "\n\n"
      , "[Optional parameters]\n"
      , "\n\n"
      , "[Common]\n"
      , "[-M|--match_file] path to the match file to use (i.e matches.f.txt or matches.f.bin)\n"
      , "[-f|--refine_intrinsic_config] Intrinsic parameters refinement option\n"
      , "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      , "\t NONE -> intrinsic parameters are held as constant\n"
      , "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
      , "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
      , "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
      , "\t -> NOTE: options can be combined thanks to '|'\n"
      , "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      ,    "\t\t-> refine the focal length & the principal point position\n"
      , "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      ,    "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
      , "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
      ,    "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
      , "[-e|--refine_extrinsic_config] Extrinsic parameters refinement option\n"
      , "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      , "\t NONE -> extrinsic parameters are held as constant\n"
      , "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions) (default: false)\n"
      , "\n\n"
      , "[Engine specifics]\n"
      , "\n\n"
      , "[INCREMENTAL]\n"
      , "\t[-a|--initial_pair_a] filename of the first image (without path)\n"
      , "\t[-b|--initial_pair_b] filename of the second image (without path)\n"
      , "\t[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      , "\t\t 1: Pinhole \n"
      , "\t\t 2: Pinhole radial 1\n"
      , "\t\t 3: Pinhole radial 3 (default)\n"
      , "\t\t 4: Pinhole radial 3 + tangential 2\n"
      , "\t\t 5: Pinhole fisheye\n"
      , "\t[--triangulation_method] triangulation method (default=" , str(triangulation_method) , "):\n"
      , "\t\t" , str(DIRECT_LINEAR_TRANSFORM) , ": DIRECT_LINEAR_TRANSFORM\n"
      , "\t\t" , str(L1_ANGULAR) , ": L1_ANGULAR\n"
      , "\t\t" , str(LINFINITY_ANGULAR) , ": LINFINITY_ANGULAR\n"
      , "\t\t" , str(INVERSE_DEPTH_WEIGHTED_MIDPOINT) , ": INVERSE_DEPTH_WEIGHTED_MIDPOINT\n"
      , "\t[--resection_method] resection/pose estimation method (default=" , resection_method , "):\n"
      , "\t\t" , str(DLT_6POINTS) , ": DIRECT_LINEAR_TRANSFORM 6Points | does not use intrinsic data\n"
      , "\t\t" , str(P3P_KE_CVPR17) , ": P3P_KE_CVPR17\n"
      , "\t\t" , str(P3P_KNEIP_CVPR11) , ": P3P_KNEIP_CVPR11\n"
      , "\t\t" , str(P3P_NORDBERG_ECCV18) , ": P3P_NORDBERG_ECCV18\n"
      , "\t\t" , str(UP2P_KUKELOVA_ACCV10)  , ": UP2P_KUKELOVA_ACCV10 | 2Points | upright camera\n"
      , "\n\n"
      , "[INCREMENTALV2]\n"
      , "\t[-S|--sfm_initializer] Choose the SfM initializer method:\n"
      , "\t\t 'EXISTING_POSE'-> Initialize the reconstruction from the existing sfm_data camera poses\n"
      , "\t\t 'MAX_PAIR'-> Initialize the reconstruction from the pair that has the most of matches\n"
      , "\t\t 'AUTO_PAIR'-> Initialize the reconstruction with a pair selected automatically\n"
      , "\t\t 'STELLAR'-> Initialize the reconstruction with a 'stellar' reconstruction\n"
      , "\t[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      , "\t\t 1: Pinhole \n"
      , "\t\t 2: Pinhole radial 1\n"
      , "\t\t 3: Pinhole radial 3 (default)\n"
      , "\t\t 4: Pinhole radial 3 + tangential 2\n"
      , "\t\t 5: Pinhole fisheye\n"
      , "\t[--triangulation_method] triangulation method (default=" , str(triangulation_method) , "):\n"
      , "\t\t" , str(DIRECT_LINEAR_TRANSFORM) , ": DIRECT_LINEAR_TRANSFORM\n"
      , "\t\t" , str(L1_ANGULAR) , ": L1_ANGULAR\n"
      , "\t\t" , str(LINFINITY_ANGULAR) , ": LINFINITY_ANGULAR\n"
      , "\t\t" , str(INVERSE_DEPTH_WEIGHTED_MIDPOINT) , ": INVERSE_DEPTH_WEIGHTED_MIDPOINT\n"
      , "\t[--resection_method] resection/pose estimation method (default=" , str(resection_method) , "):\n"
      , "\t\t" , str(DLT_6POINTS) , ": DIRECT_LINEAR_TRANSFORM 6Points | does not use intrinsic data\n"
      , "\t\t" , str(P3P_KE_CVPR17) , ": P3P_KE_CVPR17\n"
      , "\t\t" , str(P3P_KNEIP_CVPR11) , ": P3P_KNEIP_CVPR11\n"
      , "\t\t" , str(P3P_NORDBERG_ECCV18) , ": P3P_NORDBERG_ECCV18\n"
      , "\t\t" , str(UP2P_KUKELOVA_ACCV10)  , ": UP2P_KUKELOVA_ACCV10 | 2Points | upright camera\n"
      , "\n\n"
      , "[GLOBAL]\n"
      , "\t[-R|--rotationAveraging]\n"
      , "\t\t 1 -> L1 minimization\n"
      , "\t\t 2 -> L2 minimization (default)\n"
      , "\t[-T|--translationAveraging]:\n"
      , "\t\t 1 -> L1 minimization\n"
      , "\t\t 2 -> L2 minimization of sum of squared Chordal distances\n"
      , "\t\t 3 -> SoftL1 minimization (default)\n"
      , "\t\t 4 -> LiGT: Linear Global Translation constraints from rotation and matches\n"
      , "[STELLAR]\n"
      , "\t[-G|--graph_simplification]\n"
      , "\t\t -> NONE\n"
      , "\t\t -> MST_X\n"
      , "\t\t -> STAR_X\n"
      , "\t[-g|--graph_simplification_value]\n"
      , "\t\t -> Number (default: " , str(graph_simplification_value) , ")")


class EINTRINSIC(Enum):
    PINHOLE_CAMERA_START = 0
    PINHOLE_CAMERA = 1         
    PINHOLE_CAMERA_RADIAL1 = 2
    PINHOLE_CAMERA_RADIAL3 = 3 
    PINHOLE_CAMERA_BROWN = 4 
    PINHOLE_CAMERA_FISHEYE = 5
    PINHOLE_CAMERA_END = 6
    CAMERA_SPHERICAL = 7

class ETranslationAveragingMethod(Enum):
    TRANSLATION_AVERAGING_L1 = 1
    TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL = 2
    TRANSLATION_AVERAGING_SOFTL1 = 3
    TRANSLATION_LIGT = 4

class ETriangulationMethod(Enum):
    DIRECT_LINEAR_TRANSFORM = 1
    L1_ANGULAR = 2
    LINFINITY_ANGULAR = 3
    INVERSE_DEPTH_WEIGHTED_MIDPOINT = 4
    DEFAULT = 5 

class Intrinsic_Parameter_Type(Enum):
  NONE                    = 1
  ADJUST_FOCAL_LENGTH     = 2
  ADJUST_PRINCIPAL_POINT  = 4
  ADJUST_DISTORTION       = 8
  ADJUST_ALL = ADJUST_FOCAL_LENGTH | ADJUST_PRINCIPAL_POINT | ADJUST_DISTORTION

class Extrinsic_Parameter_Type(Enum):
  NONE                = 1   
  ADJUST_ROTATION     = 2
  ADJUST_TRANSLATION  = 4
  ADJUST_ALL = ADJUST_ROTATION | ADJUST_TRANSLATION

class ERotationAveragingMethod(Enum):
  ROTATION_AVERAGING_L1 = 1
  ROTATION_AVERAGING_L2 = 2

class ETranslationAveragingMethod(Enum):
  TRANSLATION_AVERAGING_L1 = 1
  TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL = 2
  TRANSLATION_AVERAGING_SOFTL1 = 3
  TRANSLATION_LIGT = 4

def isValid_ETriangulationMethod(method:ETriangulationMethod):
    method = ETriangulationMethod(method);
    isValid =  method.value >= ETriangulationMethod.DIRECT_LINEAR_TRANSFORM.value and method.value <= ETriangulationMethod.DEFAULT.value
    
    if(isValid is False):
        print("Invalid triangulation method")
    
    return isValid

def isValid_EINTRINSIC(eintrinsic:EINTRINSIC):
    isValid =  isPinhole(eintrinsic) or isSpherical(eintrinsic)

    if(isValid is False):
        print("Invalid camera type")

    return isValid

def isValid_Intrinsic_Parameter_Type(intrinsic_refinement_options:Intrinsic_Parameter_Type,check_optiion:Intrinsic_Parameter_Type):
    isValid = (intrinsic_refinement_options != check_optiion)

    if(isValid is False):
        print("Invalid input for Bundle Adjustment Intrinsic parameter refinement option")

    return isValid

def isValid_Extrinsic_Parameter_Type(extrinsic_refinement_options:Extrinsic_Parameter_Type,check_optiion:Extrinsic_Parameter_Type):
    isValid = (extrinsic_refinement_options != check_optiion)

    if(isValid is False):
        print("Invalid input for the Bundle Adjustment Extrinsic parameter refinement option")

    return isValid

def isValid_SfM_Initializer_Option(StringToEnum_Result):
    isValid = StringToEnum_Result

    if(isValid is False):
        print("Invalid input for the SfM initializer option")

    return isValid

def isValid_SfM_Engine_Type(StringToEnum_Result):
    isValid = StringToEnum_Result

    if(isValid is False):
        print("Invalid input for the SfM Engine type")

    return isValid

def isValid_ERotationAveragingMethod(rotation_averaging_method:ERotationAveragingMethod):
    rotation_averaging_method = ERotationAveragingMethod(rotation_averaging_method);
    isValid =  rotation_averaging_method.value >= ERotationAveragingMethod.ROTATION_AVERAGING_L1.value and rotation_averaging_method.value <= ERotationAveragingMethod.ROTATION_AVERAGING_L2.value
    
    if(isValid is False):
        print("Rotation averaging method is invalid")
    
    return isValid

def isValid_ETranslationAveragingMethod(translation_averaging_method:ETranslationAveragingMethod):
    translation_averaging_method = ETranslationAveragingMethod(translation_averaging_method)
    isValid = translation_averaging_method.value >= ETranslationAveragingMethod.TRANSLATION_AVERAGING_L1.value and translation_averaging_method.value <= ETranslationAveragingMethod.TRANSLATION_LIGT.value

    if(isValid is False):
        print("Translation averaging method is invalid")

    return isValid

def isValid_EGraphSimplification(StringToEnum_EGraphSimplification_Result):
    isValid =  StringToEnum_EGraphSimplification_Result
    
    if(isValid is False):
        print("Cannot recognize graph simplification method")
    
    return isValid

def isValid_Graph_Simplification_Value(graph_simplification_value):
    isValid =  graph_simplification_value > 1
    
    if(isValid is False):
        print("graph simplification value must be > 1")
    
    return isValid

def isValid_Output_Directory(directory_output_empty):
    isValid =  directory_output_empty is False
    
    if(isValid is False):
        print("It is an invalid output directory")
    
    return isValid

def isPinhole(eintrinsic:EINTRINSIC):
  eintrinsic = EINTRINSIC(eintrinsic)
  return eintrinsic.value > EINTRINSIC.PINHOLE_CAMERA_START.value and eintrinsic.value < EINTRINSIC.PINHOLE_CAMERA_END.value

def isSpherical(eintrinsic:EINTRINSIC):
  eintrinsic = EINTRINSIC(eintrinsic)
  return eintrinsic.value == EINTRINSIC.CAMERA_SPHERICAL.value

if __name__ == "__main__":
	a = EINTRINSIC.PINHOLE_CAMERA_END
	print(isPinhole(0))