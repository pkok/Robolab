clear; clc;
% params = The structure created by one of the camera initialization 
%          scripts containing the tunable parameter values for the various 
%          processing functions set appropriately for a specific camera.
%          The fields of the structure are: (See the initialization scripts
%          for more detailed descriptions of each field and their role in
%          the process.)
%
%   .xRange = Forward distance viewing range (in meters) for the IPM image.
%
%   .yRange = Side-to-side distance viewing range (in meters) for the IPM image.
%
%   .mIPM = Number of rows in the IPM image.
%
%   .step = The physical distance between pixels in the IPM image (in meters).
%
%   .stripeWidth = The physical width (in meters) of the stripes the
%                  functions are trying to find.
%
%   .filterSizeMultiplier = Factor determining the size of the smoothing
%                           derivative kernels relative to the stripe size.
%
%   .LnegThreshMultiplier = Factor determining the cutoff level for
%                           deciding which pixels have a significantly-
%                           negative Laplacian value.
%
%   .extendLength = Total number of times the stripe width along which to 
%                   search for zero crossings in the gradient.
%
%   .signChangeMultiplier = Factor determining the cutoff level for
%                           deciding which pixels have a significantly-
%                           strong zero crossing.
%
%   .minFitLength = Minimum length of lines to be fit.
%
%   .maxFitFailures = Maximum number of fruitless iterations before the
%                     line fitting algorithm gives up.
%
%   .nRandomPoints = Number of random points used in fitting each candidate
%                    line.
%
%   .nLinesEachLoop = Number of candidate lines generated for comparison
%                     with each loop of the line fitting algorithm.
%
%   .polyOrder = Order of the polynomials fit to the lines.
%
%   .fitDistThresh = Distance threshold for determining if points belong to
%                    a fitted line.
%
%   .thetaThresh = Orientation threshold for determining if points belong to
%                  a fitted line.
%
% data = The structure created by one of the camera initialization scripts 
%        containing the static data elements needed to process incoming
%        images for a specific camera and stripe size.  The fields of the
%        structure are:
%
%   .xGrid = a 1 x Nx vector giving the x values corresponding to the  
%            columns of the inverse-perspective-mapped image.  xGrid(1)
%            gives the x coordinate corresponding to the first column and
%            these increase towards the right side of the image as the
%            column numbers increase.
%
%   .yGrid = a 1 x Ny vector giving the y values corresponding the rows of
%            the inverse-perspective-mapped image.  yGrid(1) gives the y
%            coordinate corresponding to the first row and these DECREASE
%            towards the bottom of the image as the row numbers INCREASE.
%
%   .interpMap = the interpolation map for the camera as returned by the
%                function getInterpMap.
%
%   .kernels = the filter kernels returned by the function getFilterKernels. 
%   
% showPlots = optional argument which turns on diagnostic plots.  All of 
%             the plots along the way are shown for showPlots == 'y'.  Only
%             the final results plot is shown for showPlots == 'f'.  All
%             other values suppress the plots.

%------------------
%% pixelsToWorld.m
%------------------
% Put in the parameters for the DARPA web cam images (since that's still
% all we have to work with).  The values are those which were developed in
% the invPerspCalibrate.m script.
camera.name = 'Nao Lower Camera';
%camera.m = 480; % Rows (height)
%camera.m = 960; % Rows (height)
camera.m = 240; % Rows (height)
%camera.n = 1280; % Columns (width)
%camera.n = 640; % Columns (width)
camera.n = 320; % Columns (width)
%camera.h = 1.5; % Distance camera was above the ground (meters)
camera.h = 0.43; % Distance camera was above the ground (meters)
%camera.alphaTot = 30*pi/180; % Total HALF viewing angle (corner to corner - radians)
camera.alphaTot = 41.8087*pi/180; % Total HALF viewing angle (corner to corner - radians)
%camera.theta0 = -0.01373524283527; % Camera tilt angle below horizontal (radians)
camera.theta0 = 20*pi/180; % Camera tilt angle below horizontal (radians)
% That's correct, a negative sign.  The camera was pointed slightly up
% relative to horizontal.

% Start with the pixel to world mapping function.
% Keep track of how long this takes to run.  Sonce it turns out to be
% really fast, average over 10 times.
tic;
for i = 1:1
    [xMap, yMap] = pixelsToWorld(camera);
end
t = toc;
t = t/10;
disp('Time required to set up the pixel to world map:');
disp(sprintf('\t%.3f seconds', t));

% Plot the xMap versus yMap to make sure that the points fan out as
% expected.  Downsample them a bit so the grid will be sparse enough to
% tell what is going on.
hF1 = figure('Name', 'xMap, yMap', 'Units', 'pixels', 'Position', ...
    [50 50 640 480]);
sampby = 50;
xPlot = xMap(:, 1:sampby:end);
yPlot = yMap(:, 1:sampby:end);
% Plot all of the columns as lines, then all of the rows.
rPlot = size(xPlot,1);
cPlot = size(xPlot,2);
for r = 1:rPlot
    line(xPlot(r, :), yPlot(r, :), 'LineStyle', '-', 'Color', 'b', ...
        'Marker', '.', 'MarkerSize', 5);
end
for c = 1:cPlot
    line(xPlot(:,c), yPlot(:,c), 'LineStyle', '-', 'Color', 'b', ...
        'Marker', '.', 'MarkerSize', 5);
end
title('Image Pixels Mapped to World Frame', 'FontSize', 12);
xlabel('x (meters)', 'FontSize', 12);
ylabel('y (meters)', 'FontSize', 12);
axis tight;
axis equal;
drawnow;

%-----------------
%% getInterpMap.m
%-----------------

% Now set up the desired xRange, yRange and step based on our observations
% about the usable area of the IPM image from the calibration images.
params.xRange = [min(xMap(:,1)), 4];
params.yRange = [-2 2];
params.mIPM = 320;
params.step = (params.yRange(2) - params.yRange(1))/(params.mIPM-1);
% Keep track of how long this takes to run. (It's slow, so only do it once.)
tic;
[xGrid, yGrid, interpMap] = getInterpMap(xMap, yMap, camera, params);
t = toc;
disp('Time required to set up the interpolation map:');
disp(sprintf('\t%.3f seconds', t));
data.interpMap = interpMap;
data.xGrid = xGrid;
data.yGrid = yGrid;

frame = struct2cell(dir('dataset_QVGA_RGB/*.png'));
track = strcat('dataset_QVGA_RGB_IPM/',frame(1,:));
frame = strcat('dataset_QVGA_RGB/',frame(1,:));


for qq = 1 : 64
    % Load up an image to test this on and run it through the getWorldImage
    % function.
    %I = imread('roadCalPic.ppm');
    I = imread(frame{qq});
    % Again, we want an idea of how fast it runs, so keep track of the elapsed
    % time.  It turns out to be pretty quick, so do it 10 times in a row and
    % average.
    tic;
    SWc = 0;
    for i = 1:10
        [Wc, Wg] = getWorldImage(I, data);
        SWc = SWc + Wc;
    end
    SWc = SWc/10;
    SWc = imrotate(SWc,90);
    t = toc;
    t = t/10;
    disp('Time required to apply the interpolation map for an RGB image:');
    disp(sprintf('\t%.3f seconds', t));
    
    imwrite(SWc, track{qq}, 'PNG');
    
    %imshow(SWc);
    %pause;
    
    %{
    % Display the resulting IPM images and set the axes to show the world
    % coordinates instead of the rows and columns.  Also show the original
    % image for comparison.
    
    hF2 = figure('Name', 'IPM results', 'Units', 'inches', 'Position', ...
        [1 1 6.5 6]);
    
    gap = 0.075;
    fs.title = 10;
    fs.label = 9;
    fs.axes = 8;
    width = (1-4*gap)/2;
    height = (1-4*gap)/2;
    top = (1-4*gap)/2+3*gap;
    bot = gap;
    left = gap;
    right = (1-4*gap)/2+3*gap;
    
    hA = axes('Units', 'normalized', 'Position', ...
        [left top width height], ...
        'FontSize', fs.axes);
    image(I);
    set(hA, 'DataAspectRatio', [1 1 1]);
    title('Original Image', 'FontSize', fs.title);
    xlabel('Column', 'FontSize', fs.label);
    ylabel('Row', 'FontSize', fs.label);
    
    hA = axes('Units', 'normalized', 'Position', ...
        [right top width height], ...
        'FontSize', fs.axes);
    image(Wc, 'XData', params.xRange, 'YData', fliplr(params.yRange));
    set(hA, 'YDir', 'normal', 'DataAspectRatio', [1 1 1]);
    title('Inverse-Perspective-Mapped Image', 'FontSize', fs.title);
    xlabel('x = Forward Distance (meters)', 'FontSize', fs.label);
    ylabel('y = Side-to-Side Distance (meters)', 'FontSize', fs.label);
    drawnow;
    
    % To get the grayscale ones to display properly on the same figure, we need
    % to create equivalent RGB versions with the intensities repeated equally
    % in each channel.
    hA = axes('Units', 'normalized', 'Position', ...
        [left bot width height], ...
        'FontSize', fs.axes);
    Ig = mean(double(I),3)/255;
    image(repmat(Ig, [1, 1, 3]));
    set(hA, 'DataAspectRatio', [1 1 1]);
    title('Original Image', 'FontSize', fs.title);
    xlabel('Column', 'FontSize', fs.label);
    ylabel('Row', 'FontSize', fs.label);
    
    hA = axes('Units', 'normalized', 'Position', ...
        [right bot width height], ...
        'FontSize', fs.axes);
    image(repmat(Wg, [1, 1, 3]), 'XData', params.xRange, ...
        'YData', fliplr(params.yRange));
    set(hA, 'YDir', 'normal', 'DataAspectRatio', [1 1 1]);
    title('Inverse-Perspective-Mapped Image', 'FontSize', fs.title);
    xlabel('x = Forward Distance (meters)', 'FontSize', fs.label);
    ylabel('y = Side-to-Side Distance (meters)', 'FontSize', fs.label);
    drawnow;
    pause;
    %}
end
