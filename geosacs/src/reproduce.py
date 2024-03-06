import numpy as np
import math

def reproduce(model, numRepro, starting, initPoint, Ratio, crossSection, strategy):
    if np.size(starting) != numRepro:
        raise ValueError('The number of starting points should be equal to the number of reproductions')
    if np.size(initPoint, 0) != numRepro:
        raise ValueError('The number of initial points should be equal to the number of reproductions')
    
    directrix = model['directrix']
    eT = model['eT']
    # eN = model['eN']
    # eB = model['eB']
    eN = model['x_corr_axes']
    eB = model['y_corr_axes']
    szd = directrix.shape[0]
    
    newTrajectories = [None] * numRepro
    for jj in range(numRepro):
        strt = starting[0, jj]
        ratio = Ratio[jj]
        newTraj = np.zeros((szd - strt, 3))
        newTraj[0, :] = initPoint[jj, :]

        PcurrL = initPoint[jj, :]
        DCMl2g = np.vstack((eN[0, :], eB[0, :], eT[0, :]))  # Local2Global DCM using current TNB
        
        for ii in range(strt, szd):
            PcurrG = np.dot(DCMl2g, (PcurrL - directrix[ii - 1, :]).reshape(-1, 1))  # PcurrL, PcurrG: current point in Local and Global res.
            # print(f"PcurrG: {PcurrG}")
            # if np.isclose(PcurrG[2,0], 0):
            #     print("zero")
            DCMl2g = np.vstack((eN[ii, :], eB[ii, :], eT[ii, :]))  # Local2Global DCM using current TNB
            DCMg2l = DCMl2g.T

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
                PnextL0 = DCMg2l.dot(PcurrG)  # PnextL: next point in Local
                # print("PnextL0 Shape", PnextL0.shape, "directrix shape = ", directrix[ii,:].reshape(3,1).shape)
                directrix_reshaped = directrix[ii,:].reshape(3,1)
                
                test_nan = np.isnan((PnextL0 / np.linalg.norm(PnextL0)).ravel())
                if np.isclose(Ratio, 0) or np.any(test_nan):
                    PnextL = directrix_reshaped
                else:
                    PnextL = Ratio * Rc[ii] * PnextL0 / np.linalg.norm(PnextL0) + directrix_reshaped
                    
                PnextL = PnextL.reshape(-1)
                newTraj[ii - strt, :] = PnextL
                PcurrL = PnextL
            
            # elif crossSection == 'spline':
            #     curves = model['Curve']
            #     PP = curves[ii]['curve']  # get the second curve
            #     centerPP = np.array([0, 0])
                
            #     Sx, Sy = intersectPointCSpline(PcurrG[:2, 0], centerPP, PP)  # find the intersection
                
            #     if len(Sx) == 2:
            #         q, d = nearestIntersection(np.array([Sx, Sy]), PcurrG[:2, 0])
            #     elif len(Sx) == 1:
            #         q = np.array([Sx, Sy])
            #         print('Only one solution found')
            #     else:
            #         print('No intersection found')
                
            #     PnextL0 = DCMg2l.dot(PcurrG)  # PnextL: next point in Local
            #     PnextL = ratio * d * PnextL0 / np.linalg.norm(PnextL0) + directrix[ii, :]
                
            #     newTraj[ii - strt, :] = PnextL
            #     PcurrL = PnextL
                
        newTraj = np.insert(newTraj, 0, initPoint[jj, :], axis=0)
        newTrajectories[jj] = newTraj
    
    return newTrajectories
