import numpy as np
from scipy import misc


# TODO def grid_of_point_count(point_cloud, n_cells):
# TODO def grid_of_max_elevation(point_cloud):


def trim_to_roi(point_cloud,roi):
    """ Remove points outside ROI."""
    inside_roi = np.max(np.absolute(point_cloud), axis=1) < roi/2
    return point_cloud[inside_roi]

if __name__ == "__main__":
    main()
