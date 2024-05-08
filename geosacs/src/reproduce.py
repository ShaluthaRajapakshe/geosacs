import numpy as np
import math

def reproduce(model, numRepro, starting, initPoint, Ratio, crossSection, strategy, direction ):
    if np.size(starting) != numRepro:
        raise ValueError('The number of starting points should be equal to the number of reproductions')
    if np.size(initPoint, 0) != numRepro:
        raise ValueError('The number of initial points should be equal to the number of reproductions')
    
    directrix = model['directrix']
    eT = model['eT']
    
    eN = model['x_corr_axes']
    eB = model['y_corr_axes']

    idx_corr_y = model["xyz_corr_y"]

    vertical_start = model["vertical_start"]
    vertical_end = model["vertical_end"]

    szd = directrix.shape[0]
    
    newTrajectories = [None] * numRepro

    dir_changed = False
    for jj in range(numRepro):
        strt = starting[0, jj]
        ratio = Ratio[jj]
        newTraj = np.zeros((szd - strt, 3))
        newTraj[0, :] = initPoint[jj, :]


        PcurrG = initPoint[jj, :]
        DCMl2g = np.vstack((eN[0, :], eB[0, :], eT[0, :]))  # Local2Global DCM using current TNB
        
        for ii in range(strt, szd):
            PcurrL = np.dot(DCMl2g, (PcurrG - directrix[ii - 1, :]).reshape(-1, 1))  # PcurrL, PcurrG: current point in Local and Global res.


            DCMl2g = np.vstack((eN[ii, :], eB[ii, :], eT[ii, :]))  # Local2Global DCM using current TNB
            DCMg2l = DCMl2g.T


            ## Here the logic is when we are going from pick side towards the place side, as soon as we found the index where the axes change has been occured,
            ## we need to change the eB direction as otherwise it is not continuous. Here we change the axis from the axis where the change happened.
            ## However, when we are coming back from place side towards the pick side, when we found the index where the axes change has happened, we will 
            ## only shift axes from the next index only. This is because now we are in the opposite side axis and from next frame onwards, we need to switch 
            ## the axes of the next frames. 

            if dir_changed == True:
                DCMl2g = np.vstack((eN[ii, :], -eB[ii, :], eT[ii, :]))  # Local2Global DCM using current TNB
                DCMg2l = DCMl2g.T

            if not vertical_start and not vertical_end:
                if (directrix[ii,:] == idx_corr_y).all() and not dir_changed:
                    if direction == 1:
                        DCMl2g = np.vstack((eN[ii, :], -eB[ii, :], eT[ii, :]))  # Local2Global DCM using current TNB
                    
                    DCMg2l = DCMl2g.T
                    dir_changed = True

            # Adapt ratio

            if strategy == "convergent" : 
                ratio_final = 0
                decay =0.0005*ii
                Ratio = (Ratio-ratio_final)*math.exp(-decay) + ratio_final
            else: 
                # print("FIXED STRATEGY")
                pass
            
            if crossSection == 'circle':
                Rc = model['Rc']

                PnextG0 = DCMg2l.dot(PcurrL)  # PnextG: next point in Global
                directrix_reshaped = directrix[ii,:].reshape(3,1)
                
                test_nan = np.isnan((PnextG0 / np.linalg.norm(PnextG0)).ravel())
                if np.isclose(Ratio, 0) or np.any(test_nan):
                    PnextG = directrix_reshaped
                else:
                    PnextG = Ratio * Rc[ii] * PnextG0 / np.linalg.norm(PnextG0) + directrix_reshaped
                
                    
                PnextG = PnextG.reshape(-1)
                newTraj[ii - strt, :] = PnextG
                PcurrG = PnextG
            
    
                
        newTraj = np.insert(newTraj, 0, initPoint[jj, :], axis=0)
        newTrajectories[jj] = newTraj
    
    return newTrajectories
