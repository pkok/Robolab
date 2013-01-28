function [xGrid, yGrid, interpMap] = getInterpMap(xMap, yMap, camera, params)

% [xGrid, yGrid, interpMap] = getInterpMap(xMap, yMap, camera, params)
%
% getInterpMap - Finds the weights needed to create an inverse-perspective-
% mapped image with evenly spaced pixels by linearly interpolating between
% intensity/color values in the original image.  The interpolation weights
% are calculated based on the locations where the image pixels map in the
% world coordinates so that they will be based on true physical distances.
% Areas in the inverse-perspective-mapped image which are not visible in
% the original image are given weights of zero so that they will be black
% in the inverse-perspective-mapped image.
%
% Inputs:
%---------
% xMap, yMap = The image to world mapping matrices output by the 
%              pixelsToWorld function. (See help pixelsToWorld.)
% camera = The structure created by one of the camera initialization
%          scripts containing the needed camera parameters. (See help
%          pixelsToWorld.) The fields used by this function are: m.
% params = the structure produced by one of the camera initialization
%          scripts. (See help processImage.)  The fields used by this
%          function are: xRange, yRange, step.
%
% Outputs:
%----------
% xGrid = a 1 x Nx vector giving the x values corresponding to the columns 
%         of the inverse-perspective-mapped image.  xGrid(1) gives the x 
%         coordinate corresponding to the first column and these increase
%         towards the right side of the image as the column numbers
%         increase.
% yGrid = a 1 x Ny vector giving the y values corresponding the rows of the
%         inverse-perspective-mapped image.  yGrid(1) gives the y
%         coordinate corresponding to the first row and these DECREASE
%         towards the bottom of the image as the row numbers INCREASE.
% interpMap = a structure defining the interpolation mapping by the
%             following fields:
%   .pixels = an Ny x Nx x 4 array where each element gives a linear index
%             to a pixel in the original image whose value is used in the
%             interpolation needed to create the inverse-perspective-mapped
%             (IPM) image.  Ny x Nx is the size of the resulting IPM image as
%             determined by xRange, yRange, and step.  For a given row and
%             column in this array, the values along the third dimension
%             are the linear indices of the pixels in the original image
%             whose weighted sum will give the interpolated value in that
%             row and column of the IPM image.
%  .weights = an Ny x Nx x 4 array giving the weights associated with each of
%             the original image pixels specified in the corresponding
%             location in the .pixels field.
%
% Usage Tips:
%-------------
% Given this output format, to obtain the intensity/color at a giving row
% and column in the IPM image:
% 1. Get the pixels from the original image in the indices specified along
%    the third dimension in the corresponding row and column of the .pixels
%    field.
% 2. Multiply their intensities/colors by the weights in the corresponding 
%    locations in the .weights field.
% 3. Sum these products to give the intensity/color at the desired row and 
%    column of the IPM image.
% 
% For a grayscale image, this can be accomplished concisely using:
% IPM = sum(orig(interpMap.pixels).*interpMap.weights, 3);
% where orig is the original image and IPM is the IPM image.  For a color
% image, simply do this for each color channel.

% Extract the needed fields of the camera and params structures.
m = camera.m;
xRange = params.xRange;
yRange = params.yRange;
step = params.step;

% Get the number of columns in the original image (same as for xMap, yMap).
n = size(xMap, 2);

% Get the number of rows in the cropped image (the size of xMap, yMap).
mCropped = size(xMap, 1);

% Setup the IPM image grid based on the supplied ranges and step size.
xGrid = xRange(1):step:xRange(2);
% Remember to do the y grid in decreasing order so that standard image
% display techniques will put the image in the right orientation.
yGrid = yRange(2):-step:yRange(1);

% Now initialize the other outputs and then start in what will be the upper
% right corner of the IPM image (row 1 column 1) and work our way through
% building up the mapping.
nRows = length(yGrid);
nCols = length(xGrid);
interpMap.pixels = ones(nRows,nCols,4);
interpMap.weights = zeros(nRows,nCols,4);
% Note that having initialized the weights to zeros and the pixels to ones,
% we don't need to do anything for IPM image pixels which are not visible
% in the original image.  This combination will already give the desired
% result of having the pixels turn out black.

xVec = xMap(:,1); % All the columns of xMap are identical, so grab one of
                  % them to search in to make things easier.
% Also grab the minimum visible x value in the original image.
xMinVis = xVec(end);

% Do the columns first since if a given x location is off the image, we
% know we don't need to process any of the y's, but the reverse isn't true.
% Since this process takes quite a while, display some messages as we go
% along.
disp('Setting up interpolation map ...');
milestones = nCols/10*(1:10);
cDisp = floor(milestones);
pcnt = 10:10:100;

for c = 1:nCols
    x = xGrid(c);
    % Check if this x coordinate is visible in the original image.
    xRow = sum( (xVec >= x).*(x >= xMinVis) );
    if xRow > 0 % Then there's a chance we can see it.
        % Get the rows that bound the point.
        [r12, r34] = getRowBounds(xRow,mCropped);

        % Now fill in any y coordinates which are visible at this x
        % location.
        for r = 1:nRows
            y = yGrid(r);
            cVec = getColBounds(y,yMap,r12,r34);
            if cVec(1) > 0 % Then it is visible
                % Convert the row, column locations we have found in the
                % xMap, yMap matrices into equivalent linear indices in the
                % original image.
                rVec = [r12, r12, r34, r34];
                iVec = getIndices(m, mCropped, n, rVec, cVec);
                
                % Now get the weights associated with each pixel.
                wVec = getWeights(x, y, rVec, cVec, xMap, yMap);
                
                % Stick these into the 3d arrays in the right places and we
                % are done with this round.
                interpMap.pixels(r, c, :) = iVec;
                interpMap.weights(r, c, :) = wVec;
            end % if c1 > 0
        end % r for loop
    end % if xRow > 0
    % Display a status indicator as we go along.
    if ismember(c, cDisp)
        i = find(cDisp == c);
        disp(['    ... ', num2str(pcnt(i)), '% complete ...']);
    end
end % c for loop

%==========================================================================
% Helper function to get the row bounds in the xMap once the x visibility
% of a point in the IPM image is established.
function [r12, r34] = getRowBounds(xRow,mCropped)
    if xRow < mCropped
        r12 = xRow + 1;
        r34 = xRow;
    else
        r12 = xRow;
        r34 = xRow - 1;
    end
    
%==========================================================================
% Helper function to check the y direction visibility of a point in the IPM
% image and return the column bounds in the yMap if it is visible.  If it
% is not visible, the bounds are all returned as zeros.
function cVec = getColBounds(y,yMap,r12,r34)
    
    cVec = zeros(1,4); % Start out pessimistically.
    
    yVec12 = yMap(r12,:);
    % (Remember column 1 will have the largest y)
    yCol12 = sum( (yVec12 >= y).*(y >= yVec12(end)) );
    
    if yCol12 > 0 % Then we should be able to see it.
        % Since we were in range on the closer x row, we know
        % we should be on the farther x row.  Double check
        % things carefully just to be sure.
        yVec34 = yMap(r34,:);
        yCol34 = sum( (yVec34 >= y).*(y >= yVec34(end)) );
        if yCol34 > 0 % Then we know for sure we can see it.
            if yCol12 < length(yVec12)
                cVec(1) = yCol12;
                cVec(2) = yCol12 + 1;
            else
                cVec(1) = yCol12 - 1;
                cVec(2) = yCol12;
            end
            if yCol34 < length(yVec34)
                cVec(3) = yCol34;
                cVec(4) = yCol34 + 1;
            else
                cVec(3) = yCol34 - 1;
                cVec(4) = yCol34;
            end
        end
    end
    
%==========================================================================
% Helper function to convert the row, column locations we have found in the
% xMap, yMap matrices into equivalent linear indices in the original image.
function iVec = getIndices(m, mCropped, n, rVec, cVec)
    % Get the shift that takes the row number in the xMap, yMap matrices
    % back to the equivalent row number in the original image.
    shift = m - mCropped;
    % Shift all the rows.
    rOrig = rVec + shift;
    % Columns remain the same.
    
    % Now covert to indices using the sub2ind function.
    iVec = sub2ind([m, n], rOrig, cVec);

%==========================================================================
% Helper function to get the interpolation weightings for a given IPM
% pixel.
function wVec = getWeights(x, y, rVec, cVec, xMap, yMap);
    % Just follow the procedure we developed for doing the bilinear
    % interpolation.
    DEBUG = 1;
    for p = 1:4
        px(p) = xMap(rVec(p),cVec(p));
        py(p) = yMap(rVec(p),cVec(p));
    end
    
    d12y = py(1) - py(2);
    d1y = py(1) - y;
    d2y = y - py(2);
    
    d34y = py(3) - py(4);
    d3y = py(3) - y;
    d4y = y - py(4);
    
    d13x = px(3) - px(1);
    d3x = px(3) - x;
    d1x = x - px(1);
    
    wVec(1) = d3x*d2y/(d13x*d12y);
    wVec(2) = d3x*d1y/(d13x*d12y);
    wVec(3) = d1x*d4y/(d13x*d34y);
    wVec(4) = d1x*d3y/(d13x*d34y);
    
    if(DEBUG)
        if(sum(wVec<0) > 0)
            figure;
            plot(px, py, 'k.', x, y, 'b*');
            drawnow;
        end
    end
