#!/usr/bin/env python

import numpy as np


def rangeBearingElevationtoXYZ(range, bearing, elevation, sigmaRange, sigmaBearing, sigmaElevation):
    ''' Calculates the XYZ coordinates of a measurement made in range, bearing and elevation, with uncertianty
    
        @param(range)               Range measurement, m
        @param(bearing)             Azimuthal angle measurement, radians, 0 Up, + CCW
        @param(elevation)           Vertical angle, radians, 0 horizontal, +Up
        
        @param(sigmaRange)          Range measurement uncertainty (1-sigma)
        @param(sigmaBearing)        Bearing measurement uncertainty (1-sigma)
        @param(sigmaDeclination)    Declination measurement uncertainty (1-sigma)
        
        Returns:
        
        (X,Cout)
        X       Nx3 vector of XYZ coordinates.
        Cout    3x3 covariance matrix.
    
    '''

    X = np.array([[range], [0.0], [0.0]])
    x = X[0][0]
    y = X[1][0]
    z = X[2][0]

    Cxyz = np.diag([0.0,sigmaRange**2,0.0])

    r = 0
    p = elevation
    h = -bearing


    sin_r = np.sin(r)
    cos_r = np.cos(r)
    sin_p = np.sin(p)
    cos_p = np.cos(p)
    sin_h = np.sin(h)
    cos_h = np.cos(h)

    # Angle covariance.
    Crph = np.diag([sigmaElevation**2,
                    0.0,git 
                    sigmaBearing**2])    

    # Build the combined covariance
    C = np.zeros((6,6))
    C[:3,:3] = Crph[:,:]
    C[3:,3:] = Cxyz[:,:]

    # Rotation Matrix.
    M = np.array([[cos_p * cos_h, sin_r * sin_p * cos_h - cos_r * sin_h, cos_r * sin_p * cos_h+sin_r * sin_h],
    [cos_p * sin_h, sin_r * sin_p * sin_h + cos_r * cos_h, cos_r * sin_p * sin_h-sin_r * cos_h],
    [-sin_p, sin_r * cos_p, cos_r * cos_p]]);

    # Build the Jacobian
    J = np.zeros((3,6))
    J[0,0] = sin_r  *  (y  *  sin_h - z  *  cos_h  *  sin_p) + cos_r  *  (y  *  cos_h  *  sin_p + z  *  sin_h)
    J[0,1] = cos_h  *  (z  *  cos_r  *  cos_p + y  *  sin_r  *  cos_p - x  *  sin_p)
    J[0,2] = z  *  cos_h  *  sin_r - (x  *  cos_p + y  *  sin_r  *  sin_p)  *  sin_h - cos_r  *  (y  *  cos_h + z  *  sin_p  *  sin_h)
    #J[0,3] = M[0,0]
    #J[0,4] = M[0,1]
    #J(1,6,:,:) = M(1,3,:,:);
    J[1,0] = cos_r  *  (y  *  sin_p  *  sin_h - z  *  cos_h) - sin_r  *  (y  *  cos_h + z  *  sin_p  *  sin_h)
    J[1,1] = (z  *  cos_r  *  cos_p + y  *  sin_r  *  cos_p - x  *  sin_p)  *  sin_h
    J[1,2] = x  *  cos_p  *  cos_h + cos_r  *  (z  *  cos_h  *  sin_p - y  *  sin_h) + sin_r  *  (y  *  cos_h  *  sin_p + z  *  sin_h)
    #J(2,4,:,:) = M(2,1,:,:)
    #J(2,5,:,:) = M(2,2,:,:)
    #J(2,6,:,:) = M(2,3,:,:)
    J[2,0] = cos_p  *  (y  *  cos_r - z  *  sin_r)
    J[2,1] = -x  *  cos_p - (z  *  cos_r + y  *  sin_r)  *  sin_p
    J[2,2] = 0.0
    #J(3,4,:,:) = M(3,1,:,:);
    #J(3,5,:,:) = M(3,2,:,:);
    #J(3,6,:,:) = M(3,3,:,:);
    J[:,3:] = M[:,:]

    Xout = np.dot(M,X)


    # Propagate the uncertainty
    #Cout = np.matmul(J,np.matmul(C,J.T))
    Cout = np.dot(J,np.dot(C,J.T))

    
    return (Xout, Cout)

