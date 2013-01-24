function [xMap, yMap] = pixelsToWorld(camera)

% [xMap, yMap] = pixelsToWorld(camera)
%
% pixelsToWorld - Creates matrices giving the x and y locations in the
% world frame where the pixels in the bottom portion of the camera image
% map by using the inverse perspective transformation equations which we
% derived.
%
% Inputs:
%---------
% camera = The structure created by one of the camera initialization
%          scripts containing the following fields which define the needed
%          camera parameters:
%
%    .m = number of rows in the full image.
%
%    .n = number of columns in the full image.
%
%    .h = the height above the ground in meters at which the optical center 
%         (pinhole) of the camera is located.
%
%    .alphaTot = the total half viewing angle of the camera (the viewing 
%                angle measured between diagonally oppositive corners of
%                the image divided by two).
%
%    .theta0 = the camera tilt angle - the angle formed between the
%              camera's optical axis and horizontal with a positive angle
%              defined such that the camera is pointed below horizontal.
%              (Note that this parameter can be difficult to measure, but
%              it may be calculated given knowledge of where the horizon
%              appears in the image and the viewing angle - see the
%              invPerspCalibrate script.)
%
% Outputs:
%----------
% xMap, yMap = p x n matrices giving the x and y coordinates where the 
%              pixels in the last p rows of the image map in the world
%              coordinate frame. For example, the bottom row of these
%              matrices give the x, y coordinates where each pixel in the
%              bottom row of the image should map.
%
% Authors:
%----------
% Eric Johnson and Randy Hamburger
% University of Utah
% CS 5320/6320 - Computer Vision 
% April 9, 2007
%
% Custom functions used:
%------------------------
% none

% Extract the fields of the camera structure for easy reference.
m = camera.m;
n = camera.n;
h = camera.h;
alphaTot = camera.alphaTot;
theta0 = camera.theta0;

%-----------------
% Parameter Setup
%-----------------

% Calculate the horizontal and vertical half-viewing angles from alphaTot
% and the image size.
den = sqrt((m-1)^2+(n-1)^2);
alpha_u = atan( (n-1)/den * tan(alphaTot) );
alpha_v = atan( (m-1)/den * tan(alphaTot) );

% Get the horizon row from theta0 add a few rows to avoid numerical issues
% from the true horizon row being way out at x = infinity.  Add 5% of the
% rows so the adjustment will scale with the image size.
rHorizon = ceil( (m-1)/2*(1 - tan(theta0)/tan(alpha_v)) + 1 ) ... real value
    + ceil(m*0.05); % adjustment

% This makes the number of rows in the cropped image:
mCropped = m-rHorizon+1;

%------------------------
% Get xMap yMap Matrices
%------------------------

% Initialize to proper size.
xMap = zeros(mCropped,n);
yMap = zeros(size(xMap));

% Use the transformation equations we derived to populate the values for
% each pixel in the cropped region of the image.
for r = 1:mCropped
    
    % We need to calculate things based on r in the original image and
    % stuff them into the output matrices in the loop index positions.
    rOrig = r + rHorizon - 1;
    rFactor = (1-2*(rOrig-1)/(m-1))*tan(alpha_v);
    num = 1 + rFactor*tan(theta0);
    den = tan(theta0) - rFactor;
    xMap(r,1:n) = h*(num/den);
    
    for c = 1:n
        num = (1-2*(c-1)/(n-1))*tan(alpha_u);
        den = sin(theta0) - rFactor*cos(theta0);
        yMap(r,c) = h*(num/den);
    end
    
end
