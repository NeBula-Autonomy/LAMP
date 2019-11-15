from __future__ import print_function

'''
Copyright Notes

Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
'''

import os
import gt_analysis
import subprocess
from time import sleep
from zipfile import ZipFile
from datetime import datetime


'''

Run this script from within the lamp/scripts directory:

    python gt_pipeline.py

'''


# ------------------------------------------------------------------
#                       Config parameters
# ------------------------------------------------------------------

# Absoute file path (where GT data will be saved and processed)
abspath = '/home/costar/.ros/'

# Saved data filenames (used for the .zip and pose graph .bag file)
# file must already exist in the above directory
filename_data = 'pose_graph_data.zip'
filename_gt = 'pose_graph_gt.zip'

launch_cmd = 'tmuxp load $(rospack find lamp)/scripts/launch_post_processing.yaml'

# ------------------------------------------------------------------


def RunGroundTruthScript():
    '''
    Runs the launch_post_processing.yaml tmuxp script, located in lamp/scripts.
    This loads the pose graph saved in [filename_data], adds the ground truth priors
    from config/GT_artifacts.yaml, and saves the result in [filename_gt].
    '''
    subprocess.call(launch_cmd, shell=True, stdout=subprocess.PIPE)


def ExtractZip(filename, destination, output_folder_name):

    data_path = abspath + filename

    print("Ready to extract zip file on path", data_path)

    # Check for the main file
    if not os.path.exists(data_path):
        print('Saved pose graph .zip file not found')
        return

    # Check for the main file
    if not os.path.exists(data_path):
        print('Saved pose graph .zip file not found')
        return

    # Extract data from zipfile
    with ZipFile(data_path, 'r') as zipObj:
        zipObj.extractall(destination)
        os.rename(destination + 'pose_graph', destination + output_folder_name)


def ProcessData():

    RunGroundTruthScript()

    # Make sure that given files are zip files
    if not filename_data.endswith('.zip'):
        print("Data must be a .zip file")

    if not filename_gt.endswith('.zip'):
        print("Ground truth must be a .zip file")

    output_dir_rel = 'groundtruth_' + datetime.now().strftime("%d-%m-%Y_%H-%M-%S" + "/")
    output_dir = abspath + output_dir_rel

    data_path = abspath + filename_data

    # Extract to folders which are the same as the given filenames but without the .zip extension
    ExtractZip('pose_graph_data.zip', output_dir, filename_data[:-4])
    ExtractZip('pose_graph_gt.zip', output_dir, filename_gt[:-4])

    full_filename_data = filename_data[:-4] + '/pose_graph.bag'
    full_filename_gt = filename_gt[:-4] + '/pose_graph.bag'

    gt_analysis.PerformAnalysis(
        output_dir, full_filename_data, full_filename_gt)


if __name__ == '__main__':

    ProcessData()
