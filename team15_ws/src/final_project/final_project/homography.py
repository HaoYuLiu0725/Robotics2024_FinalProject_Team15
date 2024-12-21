import numpy as np

def solve_homography(u, v):
    """
    This function should return a 3-by-3 homography matrix,
    u, v are N-by-2 matrices, representing N corresponding points for v = T(u)
    :param u: N-by-2 source pixel location matrices
    :param v: N-by-2 destination pixel location matrices
    :return:
    """
    N = u.shape[0]
    H = None

    if v.shape[0] is not N:
        print('u and v should have the same size')
        return None
    if N < 4:
        print('At least 4 points should be given')

    # TODO: 1.forming A

    # Solution 2:
    # h_11*u_x + h_12*u_y + h_13 - h_31*u_x*v_x - h_32*u_y*v_x - h_33*v_x = 0
    # h_21*u_x + h_22*u_y + h_23 - h_31*u_x*v_y - h_32*u_y*v_y - h_33*v_y = 0
    #                                                           [h_11
    #                                                            h_12
    #                                                            h_13
    #                                                            h_21
    # [u_x  u_y   1    0    0   0  -u_x*v_x   -u_y*v_x  -v_x  X  h_22  = [0]
    #    0    0   0  u_x  u_y   1  -u_x*v_y   -u_y*v_y  -v_y]    h_23
    #                                                            h_31
    #                                                            h_32
    #                                                            h_33]

    u_x = u[:,0].reshape((N,1))
    u_y = u[:,1].reshape((N,1))
    v_x = v[:,0].reshape((N,1))
    v_y = v[:,1].reshape((N,1))
    
    equation_1 = np.concatenate((u_x, u_y, np.ones((N,1)), np.zeros((N,3)), -1 * np.multiply(u_x, v_x), -1 * np.multiply(u_y, v_x), -1 * v_x), axis=1 )
    equation_2 = np.concatenate((np.zeros((N,3)), u_x, u_y, np.ones((N,1)),  -1 * np.multiply(u_x, v_y), -1 * np.multiply(u_y, v_y), -1 * v_y), axis=1 )
    A = np.concatenate((equation_1, equation_2))

    # TODO: 2.solve H with A
    U, S, Vh = np.linalg.svd(A)
    H = np.transpose(Vh)[:, -1]
    H = H.reshape(3,3)

    return H


def warping(src, dst, H, ymin, ymax, xmin, xmax, direction='b'):
    """
    Perform forward/backward warpping without for loops. i.e.
    for all pixels in src(xmin~xmax, ymin~ymax),  warp to destination
        (xmin=0,ymin=0)  source                       destination
                        |--------|              |------------------------|
                        |        |              |                        |
                        |        |     warp     |                        |
    forward warp        |        |  --------->  |                        |
                        |        |              |                        |
                        |--------|              |------------------------|
                                (xmax=w,ymax=h)

    for all pixels in dst(xmin~xmax, ymin~ymax),  sample from source
                            source                       destination
                        |--------|              |------------------------|
                        |        |              | (xmin,ymin)            |
                        |        |     warp     |           |--|         |
    backward warp       |        |  <---------  |           |__|         |
                        |        |              |             (xmax,ymax)|
                        |--------|              |------------------------|

    :param src: source image
    :param dst: destination output image
    :param H:
    :param ymin: lower vertical bound of the destination(source, if forward warp) pixel coordinate
    :param ymax: upper vertical bound of the destination(source, if forward warp) pixel coordinate
    :param xmin: lower horizontal bound of the destination(source, if forward warp) pixel coordinate
    :param xmax: upper horizontal bound of the destination(source, if forward warp) pixel coordinate
    :param direction: indicates backward warping or forward warping
    :return: destination output image
    """

    h_src, w_src, ch = src.shape
    h_dst, w_dst, ch = dst.shape
    H_inv = np.linalg.inv(H)

    # TODO: 1.meshgrid the (x,y) coordinate pairs
    x_coord, y_coord = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))

    # TODO: 2.reshape the destination pixels as N x 3 homogeneous coordinate
    pixels_coord = np.stack((x_coord.flatten(), y_coord.flatten(), np.ones_like(x_coord.flatten())), axis=0)

    if direction == 'b':
        # TODO: 3.apply H_inv to the destination pixels and retrieve (u,v) pixels, then reshape to (ymax-ymin),(xmax-xmin)
        dst_coord = np.dot(H_inv, pixels_coord)
        dst_x = np.divide(dst_coord[0,:], dst_coord[-1,:])
        dst_y = np.divide(dst_coord[1,:], dst_coord[-1,:])
        dst_x = np.round(dst_x.reshape((ymax - ymin), (xmax - xmin))).astype(int)
        dst_y = np.round(dst_y.reshape((ymax - ymin), (xmax - xmin))).astype(int)

        # TODO: 4.calculate the mask of the transformed coordinate (should not exceed the boundaries of source image)
        mask = (dst_x >=0) & (dst_x < w_src) & (dst_y >= 0) & (dst_y < h_src)

        # TODO: 5.sample the source image with the masked and reshaped transformed coordinates
        mask_dst_x = dst_x[mask]
        mask_dst_y = dst_y[mask]

        # TODO: 6. assign to destination image with proper masking
        dst[y_coord[mask], x_coord[mask]] = src[mask_dst_y, mask_dst_x]     

    elif direction == 'f':
        # TODO: 3.apply H to the source pixels and retrieve (u,v) pixels, then reshape to (ymax-ymin),(xmax-xmin)
        dst_coord = np.dot(H, pixels_coord)
        dst_x = np.divide(dst_coord[0,:], dst_coord[-1,:])
        dst_y = np.divide(dst_coord[1,:], dst_coord[-1,:])
        dst_x = np.round(dst_x.reshape((ymax - ymin), (xmax - xmin))).astype(int)
        dst_y = np.round(dst_y.reshape((ymax - ymin), (xmax - xmin))).astype(int)

        # TODO: 4.calculate the mask of the transformed coordinate (should not exceed the boundaries of destination image)
        mask = (dst_x >=0) & (dst_x < w_dst) & (dst_y >= 0) & (dst_y < h_dst)

        # TODO: 5.filter the valid coordinates using previous obtained mask
        mask_dst_x = dst_x[mask]
        mask_dst_y = dst_y[mask]

        # TODO: 6. assign to destination image using advanced array indicing
        dst[mask_dst_y, mask_dst_x] = src[y_coord[mask], x_coord[mask]]

    return dst

