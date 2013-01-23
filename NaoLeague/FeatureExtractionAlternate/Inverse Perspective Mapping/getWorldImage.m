function [Wc, Wg] = getWorldImage(I, data)

% [Wc, Wg] = getWorldImage(I, data)
%
% getWorldImage - Applies the inverse-perspective interpolation mapping to
% an RBG camera image and returns a color and grayscale
% inverse-perspective-mapped image.
%
% Inputs:
%---------
% I = m x n x 3 uint8 RGB image as captured by the camera with intensities
%     for each color channel ranging from 0 to 255.
% data = the structure produced by one of the camera initialization
%        scripts. (See help processImage.)  The fields used by this
%        function are: interpMap.
%
% Outputs:
%----------
% Wc = Ny x Nx x 3 double RGB inverse-perspective-mapped image of the road
%      over the x and y range defined by the parameters provided at the
%      time the interpMap structure was created.
% Wg = Ny x Nx double grayscale version of Wc.
%
% Author:
%----------
% Eric Johnson
% University of Utah
% CS 6320 - Computer Vision 
% April 14, 2007
%
% Custom functions used:
%------------------------
% none

% Extract the interpolation map.
interpMap = data.interpMap;

% First get the color version.
[nRows, nCols, dummy] = size(interpMap.pixels);
Wc = zeros(nRows, nCols, 3, 'double');
for c = 1:3
    Id = double(I(:,:,c))/255;
    Wc(:,:,c) = sum(Id(interpMap.pixels).*interpMap.weights, 3);
end

% A little confusing debugging revealed that roundoff error can cause very
% bright elements of Wc to exceed 1 by around 1e-16 which, sadly enough,
% causes errors when you try to display the image.  So, go ahead and make
% sure the elements of Wc are clipped to EXACTLY 0 and 1.
Wc = min(Wc, 1.0000);
Wc = max(Wc, 0.0000);

% Now average the intensities in the channels to get the grayscale version.
% (I compared the result of this simple operation with the output of the
% rgb2gray function - which is significantly more complex - and didn't
% notice any appreciable difference, so let's keep it simple.)
Wg = mean(Wc, 3);