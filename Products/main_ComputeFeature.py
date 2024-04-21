import argparse
import sys
import os
import json

#75
EXIT_FAILURE = 1

sSFM_Data_Filename = ""
sOutDir = ""
bUpRight = False
sImage_Describer_Method = "SIFT"
bForce = False
sFeaturePreset = ""

# 定义特征描述器的预设质量级别枚举
class EDESCRIBER_PRESET:
    NORMAL_PRESET = 0
    HIGH_PRESET = 1
    ULTRA_PRESET = 2

# 将字符串转换为特征描述器预设质量级别枚举值的函数




def save_describer_configuration(image_describer, output_file):
    # 保存图像描述器的配置到 JSON 文件
    with open(output_file, "w") as f:
        json.dump({"image_describer": image_describer}, f)
      

def string_to_enum(sPreset):
    if sPreset == "NORMAL":
        return EDESCRIBER_PRESET.NORMAL_PRESET
    elif sPreset == "HIGH":
        return EDESCRIBER_PRESET.HIGH_PRESET
    elif sPreset == "ULTRA":
        return EDESCRIBER_PRESET.ULTRA_PRESET
    else:
        return -1


def log_error(message):
    print("ERROR:", message)


def log_info(message):
    print("INFO:", message)

def display_call_info(args, iNumThreads=None):
    info = f"You called:\n{sys.argv[0]}\n"
    info += f"--input_file {args.sSfM_Data_Filename}\n"
    info += f"--outdir {args.sOutDir}\n"
    info += f"--describerMethod {args.sImage_Describer_Method}\n"
    info += f"--upright {args.bUpRight}\n"
    info += f"--describerPreset {args.sFeaturePreset if args.sFeaturePreset else 'NORMAL'}\n"
    #用户选择的描述子预设值，默认为NORMAL
    info += f"--force {args.bForce}\n"
    
    if iNumThreads is not None:  #检查OpenMP是否是可调用的，线程数是否已经给出
        info += f"--numThreads {iNumThreads}\n"
    
    log_info(info)




def setup_arg_parser():
        # 创建命令行解析器
    parser = argparse.ArgumentParser(description="Compute image features and descriptors")
    
    # 添加必需的命令行选项
    parser.add_argument("-i", "--input_file", required=True, help="Input SfM_Data file")
    parser.add_argument("-o", "--outdir", required=True, help="Output directory")

    # 添加可选的命令行选项
    parser.add_argument("-m", "--describerMethod", default="SIFT", help="Method to describe an image (default: SIFT)")
    parser.add_argument("-u", "--upright", action="store_true", help="Use Upright feature")
    parser.add_argument("-f", "--force", action="store_true", help="Force to recompute data")
    parser.add_argument("-p", "--describerPreset", default="NORMAL", help="Preset for image describer configuration (default: NORMAL)")

    # 如果编译时启用了 OpenMP，则添加线程数选项
    try:#121
        import omp
        parser.add_argument("-n", "--numThreads", type=int, help="Number of parallel computations")
    except ImportError:
        print("OpenMP is not available, proceeding without parallel computation options.")
    
    return parser
    
#从文件中动态加载图像描述器（将恢复旧的使用设置）
def load_image_describer_from_file(sImage_describer, image_describer):
    if not os.path.exists(sImage_describer):
        return False

    try:
        with open(sImage_describer, "r") as f:
            data = json.load(f)
            # 假设 JSON 数据中包含图像描述器的配置信息
            image_describer.update(data["image_describer"])
            return True
    except Exception as e:
        print(f"ERROR: {e}\nCannot dynamically allocate the Image_describer interface.", file=sys.stderr)
        return False    

def main():
    parser = setup_arg_parser()
    # 解析命令行参数
    args = parser.parse_args()
    
    #输出错误信息
    if len (sys.agrv) == 1:#检查命令行参数的数量。如果只有一个参数，则抛出一个异常
       error_message = "Invalid command line parameter."
       log_error(error_message)
       #cmd.process(argc, argv);#129 如果命令行参数的数量大于1，则调用cmd.process函数处理这些参数
       print_error(sys.agrv[0])
       sys.exit(EXIT_FAILURE)
    
    #日志输出调用信息
    display_call_info(args, iNumThreads=None)
    
    #检查输出目录
    if not sOutDir:
        log_error("\nIt is an invalid output directory")
        sys.exit(EXIT_FAILURE)
    if not os.path.exits(sOutDir): #输出目录是否存在
        try:
            os.makedirs(sOutDir)#创建输出目录
        except OSError as e:
            log_error(f"Cannot create output directory:{e}")
            sys.exit(EXIT_FAILURE)
    

#211
    image_describer = Image_describer()
    sImage_describer = os.path.join(sOutDir, "image_describer.json")
    # 检查是否存在并且不需要强制重新计算
    if not bForce and os.path.isfile(sImage_describer):
        # 从文件中动态加载图像描述器
        if not load_image_describer_from_file(sImage_describer, image_describer):
            sys.exit(EXIT_FAILURE)
    
#264
    if not image_describer:
        print(f"ERROR: Cannot create the designed Image_describer: {sImage_Describer_Method}.", file=sys.stderr)
        sys.exit(EXIT_FAILURE)
    

    
    
            
def print_error(program_name):
    usage_info = f"""
    Usage: {program_name}
    [-i|--input_file] a SfM_Data file
    [-o|--outdir path]

    [Optional]
    [-f|--force] Force to recompute data
    [-m|--describerMethod]
    (method to use to describe an image):
    SIFT (default),
    SIFT_ANATOMY,
    AKAZE_FLOAT: AKAZE with floating point descriptors,
    AKAZE_MLDB:  AKAZE with binary descriptors
    [-u|--upright] Use Upright feature 0 or 1
    [-p|--describerPreset]
    (used to control the Image_describer configuration):
    NORMAL (default),
    HIGH,
    ULTRA: !!Can take long time!!
    """
    print({usage_info})
    



if __name__ == "__main__":
    main()
