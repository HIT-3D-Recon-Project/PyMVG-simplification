import argparse
import os
EXIT_FAILURE = 0

def read_datafile():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d' , '--database', help = 'the sensor width database PATH')
    
    args = parser.parse_args()
    datafile = args.database

    if(os.path.exists(datafile)):
        print("The data file doesn't exist")
        return EXIT_FAILURE
    else:
        return datafile

def read_input():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i' , '--input', help = 'the images PATH')

    args = parser.parse_args()
    imgdir = args.input
    
    if(os.path.exists(imgdir)):
        print("The input directory doesn't exist")
        return EXIT_FAILURE
    else:
        return imgdir

def read_output():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o' , '--output', help = 'the output PATH')

    args = parser.parse_args()
    outdir = args.output

    try:
        if not os.path.exists(outdir):
            raise Exception("The output directory doesn't exist, try create a new one")   #不是，在这里raise有用吗，应该扔到log去
        else:
            return outdir
    except Exception as e:
        if not os.makedirs(outdir):
            print("Invalid path: cannot create output directory")   #那就再试试print（
            return EXIT_FAILURE
        else:
            return outdir

def search_size(model, datadir):
    database = open(datadir, 'r')
    data = database.readlines()

    for line in data:
        parts = line.split(";")
        if parts[0] == model:
            return parts[1]
    return 0

def files_name(dir):
    files = os.listdir(dir)
    files.sort()
    return files

def help_log():
    return '''
[-i|--imageDirectory]\n
[-d|--sensorWidthDatabase]\n
[-o|--outputDirectory]\n
'''
            