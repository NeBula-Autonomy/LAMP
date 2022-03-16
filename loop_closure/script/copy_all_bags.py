
import os
import tqdm
from shutil import copyfile

from_location = "/media/chris/hdd2/BenchmarkDatasets"
to_location = "/media/chris/hdd3/more_bags"
i = 0
for root, dirs, files in tqdm.tqdm(list(os.walk(from_location))):
    for file in files:
        if file == "pose_graph_opt.bag":
            name = root.split("/")[-1]
            os.mkdir(os.path.join(to_location , name+str(i)))
            copyfile(os.path.join(root,file),os.path.join(to_location,name+str(i),file))
            i+=1
