
import numpy as np
from randomPointsInSpatialCircle import randomPointsInSpatialCircle

def randomInitialPoints(encodedModel, numRepro, crossSectionType):
    if crossSectionType == 'circle':
        strt = 0  # Adjust index if needed based on MATLAB 1-based indexing
        Rc = encodedModel['Rc'][strt]
        # eN = encodedModel['eN'][strt]
        # eB = encodedModel['eB'][strt]
        eN = encodedModel['x_corr_axes'][strt]
        eB = encodedModel['y_corr_axes'][strt]
        C = encodedModel['directrix'][strt]
        initPoint, Ratio = randomPointsInSpatialCircle(numRepro, Rc, C, eN, eB)
        
    # elif crossSectionType == 'spline':
    #     strt = 0  # Adjust index if needed based on MATLAB 1-based indexing
    #     curve = encodedModel['Curve'][strt]['curve']
    #     C = encodedModel['directrix'][strt]
    #     eN = encodedModel['eN'][strt]
    #     eB = encodedModel['eB'][strt]
    #     eT = encodedModel['eT'][strt]
    #     initPoint, Ratio = randomPointsInSpatialBSpline(numRepro, curve, C, eT, eN, eB)
    
    else:
        raise ValueError('Cross-section type unknown!')
    
    return initPoint, Ratio