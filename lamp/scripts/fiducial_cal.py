'''
Add bash alias to .bashrc, e.g.:

    alias fc='python3  ~/$WS/src/husky_core/localizer_blam/src/lamp/scripts/fiducial_cal.py'


Then run this command in the directory of the file _husky3_calibration_from_file.log:

    fc husky3


Parses _husky3_fiducial_calibration_from_file.log and writes the calibration
to ~/.ros/fiducial_calibration_husky3.yaml.

To write a zero calibration to the file, use

    fc husky3 zero
'''


import sys


class Position(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Orientation(object):
    def __init__(self):
        self.w = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Calibration(object):
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()

    def __str__(self):
        string = 'orientation:\n'
        string += '  w: {}\n'.format(self.orientation.w)
        string += '  x: {}\n'.format(self.orientation.x)
        string += '  y: {}\n'.format(self.orientation.y)
        string += '  z: {}\n'.format(self.orientation.z)
        string += 'position:\n'
        string += '  x: {}\n'.format(self.position.x)
        string += '  y: {}\n'.format(self.position.y)
        string += '  z: {}'.format(self.position.z)
        return string


def GetValue(line):
    return float(line.split('=')[1])


def ReadInput(infile):

    print('\nInput file (./{}):\n'.format(infile))

    cal = Calibration()

    # Read from calibration file
    lines = []

    with open(infile) as f:
        for line in f:
            if 'fiducial_calibration/' not in line:
                continue
            print(line.rstrip())
            # print(GetValue(line))
            lines.append(line.rstrip())

    # Store calibration input into Calibration object
    for line in lines:
        if 'position/x' in line:
            cal.position.x = GetValue(line)
        elif 'position/y' in line:
            cal.position.y = GetValue(line)
        elif 'position/z' in line:
            cal.position.z = GetValue(line)
        elif 'orientation/x' in line:
            cal.orientation.x = GetValue(line)
        elif 'orientation/y' in line:
            cal.orientation.y = GetValue(line)
        elif 'orientation/z' in line:
            cal.orientation.z = GetValue(line)
        elif 'orientation/w' in line:
            cal.orientation.w = GetValue(line)

    return cal


def WriteOutput(outfile, cal):

    print('\n\nOutput file: ({})\n\n{}'.format(outfile, cal.__str__()))
    print('\nWriting calibration to file: ' + outfile)

    with open(outfile, 'w') as f:
        f.write(cal.__str__())


def main():

    args = len(sys.argv) - 1
    if args < 1:
        print("Please provide a robot name")
        return

    # Read the robot name from command line args
    robot = sys.argv[1]

    # Optional command line arg specifying default calibration
    zero = False
    if args == 2 and sys.argv[2] == 'zero':
        zero = True

    infile = '_' + robot + '_fiducial_calibration_from_file.log'
    outfile = '/home/costar/.ros/fiducial_calibration_' + robot + '.yaml'

    # Load calibration
    if zero:
        cal = Calibration()
    else:
        cal = ReadInput(infile)

    # Write to output file
    WriteOutput(outfile, cal)


if __name__ == '__main__':
    main()    # Read from calibration file
