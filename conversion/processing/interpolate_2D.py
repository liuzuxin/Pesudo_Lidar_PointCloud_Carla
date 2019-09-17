import numpy as np
def interpolate_2D(V, x_query, y_query, threshold):
    """Conditioned interpolation between pixel values in the queried points.

    Let q be the pixel that the query point lies in. If the difference in pixel
    value between q and the three neighboring pixels closest to the query point
    is larger than threshold, then no interpolation is performed; the value in q
    is used."""
    # Four pixels with indices cX and cY:
    # (x1,y1) (x2,y1)
    # (x1,y2) (x2,y2)

    n_points = len(x_query)
    interpolated_values = np.zeros([n_points,1])
    height = np.size(V,0)
    width = np.size(V,1)

    # For each point
    for i_point in range(n_points):

        # Get the query point coordinates
        x = x_query[i_point]
        y = y_query[i_point]

        bound_L = x < 0.5                   # Are we at left boundary?
        bound_R = x > np.size(V,1) - 0.5    # Are we at right boundary?
        bound_T = y < 0.5                   # Are we at upper boundary?
        bound_B = y > np.size(V,0) - 0.5    # Are we att lower boundary?

        # Get the pixel index which the query point lies in
        xp, yp, x1, x2, y1, y2 = 0, 0, 0, 0, 0, 0

        if bound_R:
            xp = width - 1
        else:
            xp = int(x)

        if bound_B:
            yp = height - 1
        else:
            yp = int(y)

        if x - xp < 0.5:
            x1 = xp - 1
            x2 = xp
        else:
            x1 = xp
            x2 = xp + 1

        if y - yp < 0.5:
            y1 = yp - 1
            y2 = yp
        else:
            y1 = yp
            y2 = yp + 1

        # The most common case is to not be on any boundary
        if not(bound_L or bound_R or bound_T or bound_B):

            # Get value in each pixel of relevance
            v1 = V[y1, x1]
            v2 = V[y1, x2]
            v3 = V[y2, x1]
            v4 = V[y2, x2]

            # Interpolate only if difference between any pair of depth
            # values does not exceed threshold.
            if max([v1,v2,v3,v4]) - min([v1,v2,v3,v4]) < threshold:
                a = v3*((x2+0.5) - x) + v4*(x - (x1+0.5))
                b = v1*((x2+0.5) - x) + v2*(x - (x1+0.5))
                c = b*((y2+0.5) - y) + a*(y - (y1+0.5))
            else:
                c = V[yp, xp]

        # If we are in a corner pixel
        elif (bound_L or bound_R) and (bound_T or bound_B):

            # Just return pixel value without interpolating
            c = V[yp, xp]

        elif bound_T: # If we are at top boundary

            # Interpolate in x only
            v3 = V[y2, x1]
            v4 = V[y2, x2]

            if max(v3,v4) - min(v3,v4) < threshold:
                c = v3*((x2+0.5) - x) + v4*(x - (x1+0.5))
            else:
                c = V[yp, xp]

        elif bound_B: # If we are at bottom boundary

            # Interpolate in x only
            v1 = V[y1, x1]
            v2 = V[y1, x2]

            if max(v1,v2) - min(v1,v2) < threshold:
                c = v1*((x2+0.5) - x) + v2*(x - (x1+0.5))
            else:
                c = V[yp, xp]

        elif bound_L: # If we are at left boundary

            # Interpolate in y only
            v2 = V[y1, x2]
            v4 = V[y2, x2]

            if max(v2,v4) - min(v2,v4) < threshold:
                c = v2*((y2+0.5) - y) + v4*(y - (y1+0.5))
            else:
                c = V[yp, xp]

        else: # We must be at right boundary

            # Interpolate in y only
            v1 = V[y1, x1]
            v3 = V[y2, x1]

            if max(v2,v4) - min(v2,v4) < threshold:
                c = v1*((y2+0.5) - y) + v3*(y - (y1+0.5))
            else:
                c = V[yp, xp]

        interpolated_values[i_point] = c

    return interpolated_values
