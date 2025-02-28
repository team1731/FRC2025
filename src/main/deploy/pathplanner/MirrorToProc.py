#!/usr/bin/python3

import os, sys, getopt

FieldWidth = 16.503396 # inches: 652.73 - 3.0
FieldHeight =  8.0518  # 317 inches

def convert_x(x):
   return FieldWidth - x

def convert_y(y):
   return FieldHeight - y

def convert_rotation(rotation):
   return rotation * -1

def produce_output(zlist, outputfile):
      with open(outputfile, 'w') as fo:
         fo.writelines(zlist)
         # add CR/LF
        # fo.write("\n")

def parse_path(inputfile):
   plist = []

   with open(inputfile) as fi:
      lines = fi.readlines()

   for linenum in range(len(lines)):
      line = lines[linenum]

      if line.__contains__('"y"'):
         parts = line.split('"y": ')
         number = parts[1].split(',')[0]
         zx = convert_y(float(number))
         plist.append(parts[0] + '"y": ' + str(zx)  + '\n')
      elif line.__contains__('"rotationDegrees"'):
         rparts = line.split('"rotationDegrees": ')
         rnumber = rparts[1].split(',')[0]
         zr = convert_rotation(float(rnumber))
         plist.append(rparts[0] + '"rotationDegrees": ' + str(zr)  + '\n')
      elif line.__contains__('"rotation"'):
         rparts = line.split('"rotation": ')
         rnumber = rparts[1].split(',')[0]
         zr = convert_rotation(float(rnumber))
         plist.append(rparts[0] + '"rotation": ' + str(zr) + '\n')
      elif line.__contains__('"linkedName"'):
         parts = line.split('"')
         if not 'null' in parts[2]:
            linkedName = 'Processor_' + parts[3].strip()
            fline = parts[0] + '"linkedName": "' + linkedName + '"\n'
         else:
            fline = line
         plist.append(fline)
      elif line.__contains__('"folder"'):
         fline = line.replace("Red", "Blu")
         plist.append(fline)
      else:
         plist.append(line)

   return plist

def parse_auto(inputfile):
   zlist = []

   with open(inputfile) as fi:
      lines = fi.readlines()

   for linenum in range(len(lines)):
      line = lines[linenum]

      if line.__contains__('"pathName"'):
         zlist.append(line.replace("Blu", "Proc_Blu"))
         print("line is" + line)
         # dirname = os.path.dirname(inputfile)
         # filename = os.path.basename(inputfile)
         # filename = filename.replace("Red", "Blu")
         # outputPath = os.path.join(dirname, filename)
         # print(outputPath)

         pathParts = line.split('"')
         fileName = pathParts[3].split('.')[0]
         print("filename" + fileName)
         pathName = os.path.join("paths", fileName + ".path")
         print('In  File: ' + pathName)
         outfilePath = os.path.join("paths",  "Proc_" + fileName + ".path")
         
         print('Out File: ' + outfilePath)
         plist = parse_path(pathName)
         produce_output(plist, outfilePath)
      elif line.__contains__('"y"'):
         parts = line.split('"y": ')
         number = parts[1].split(',')[0]
         zx = convert_y(float(number))
         zlist.append(parts[0] + '"x": ' + str(zx) + ',' + '\n')
      elif line.__contains__('"rotation"'):
         parts = line.split('"rotation": ')
         number = parts[1].split(',')[0]
         zx = convert_rotation(float(number)) #180.0 + float(number)
         zlist.append(parts[0] + '"rotation": ' + str(zx) + '\n')
      else:
         zlist.append(line)

   return zlist

def main(argv):
   inputfile = None
   outputfile = None
   try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print ('MirrorToProc.py -i <inputfile> -o <outputfile>')
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print ('MirrorToProc.py -i <inputfile> -o <outputfile>')
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
   if inputfile:
      if not outputfile:
         dirname = os.path.dirname(inputfile)
         filename = os.path.basename(inputfile)
         outputfile = os.path.join(dirname, filename)

      print ('Input file is:  ', inputfile)
      print ('Output file is: ', outputfile)
      zlist = parse_auto(inputfile)
      produce_output(zlist, outputfile)
   else:
      print("Nothing Done.")

if __name__ == "__main__":
   main(sys.argv[1:])