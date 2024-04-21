import argparse
import os
import argparse
EXIT_FAILURE = 0

def read_datafile():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', help='the sensor width database PATH')
    args = parser.parse_args()
    datafile = args.database

    if not os.path.exists(datafile):
        print("The data file doesn't exist")
        return "ERROR"
    else:
        return datafile

def read_input():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', help='the images PATH')
    args = parser.parse_args()
    imgdir = args.input

    if not os.path.exists(imgdir):
        print("The input directory doesn't exist")
        return "ERROR"
    else:
        return imgdir

def read_output():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', help='the output PATH')
    args = parser.parse_args()
    outdir = args.output

    if not os.path.exists(outdir):
        try:
            os.makedirs(outdir)
            return outdir
        except Exception as e:
            print("Invalid path: cannot create output directory")
            return "ERROR"
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
            