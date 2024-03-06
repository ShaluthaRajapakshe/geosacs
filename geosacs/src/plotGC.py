import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_GC(model, demos, numSet, numDemos, gcPlotType, crossSectionType, fhandle=None):
    directrix = model['directrix']
    GC = model['GC']

    print("GC shape", GC.shape)

    if fhandle is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig = fhandle
        ax = fig.gca(projection='3d')

    stepc = 1
    if crossSectionType == 'spline':
        stepc = int(0.01 * len(directrix[:, 0]))  # 10% of the data is used

    if gcPlotType == 'line':
        print('In plotting', GC.shape)
        for kk in range(0, GC.shape[0], stepc):
            C = GC[kk, :, :]
            print("C shape", C.shape)
            ax.plot3D(C[:, 0], C[:, 1], C[:, 2], color='k')  # [0.5, 0.5, 0.5]

    elif gcPlotType == 'surface':
        X = GC[:, :, 0]
        Y = GC[:, :, 1]
        Z = GC[:, :, 2]
        ax.plot_surface(X, Y, Z, cmap='winter', alpha=0.5, shade=True)
        # ax.plot_wireframe(X, Y, Z, alpha=0.5, cmap='winter') 
    else:
        raise ValueError('Unknown plot type!')



        # for kk in range(0, GC.shape[2], stepc):
        #     C = GC[:, :, kk]
        #     ax.plot3D(C[0, :], C[1, :], C[2, :], color='k')  # [0.5, 0.5, 0.5]

    # elif gcPlotType == 'surface':
    #     X = GC[0, :, ::stepc]
    #     Y = GC[1, :, ::stepc]
    #     Z = GC[2, :, ::stepc]
    #     ax.plot_surface(X, Y, Z, cmap='winter', alpha=0.5, shade=True)
    #     # ax.plot_wireframe(X, Y, Z, alpha=0.5, cmap='winter') 
    # else:
    #     raise ValueError('Unknown plot type!')

    ax.plot3D(directrix[:, 0], directrix[:, 1], directrix[:, 2], 'b', linewidth=2)

    for ii in range(numDemos):
        print(demos.shape, demos[ii].shape)
        ax.plot3D(demos[:,ii,0], demos[:,ii,1], demos[:,ii,2], 'r', linewidth=1)
        # ax.plot3D(demos[ii][:, 0], demos[ii][:, 1], demos[ii][:, 2], 'r', linewidth=1)

    ax.set_title(f'Set {numSet}')
    ax.set_xlabel('x_1')
    ax.set_ylabel('x_2')
    ax.set_zlabel('x_3')
    ax.view_init(19.6, -6.8)
    ax.grid(True)
    plt.tight_layout()

    for child in fig.get_children():
        if isinstance(child, plt.Axes):
            child.set_xlabel('x_1')
            child.set_ylabel('x_2')
            child.set_zlabel('x_3')
            child.tick_params(axis='both', which='major', labelsize=12)
            child.tick_params(axis='both', which='minor', labelsize=10)
            child.set_box_aspect([1, 1, 1])  # Set aspect ratio to equal

    plt.show()
