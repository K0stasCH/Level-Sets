clc; close all; clear all;

% run('SDF.m');     %% not using NN
load('SDF (9).mat');    %% using NN

SDF3D = SDF;

in = SDF3D;
out = SDF3D;

sz = size(SDF3D);

% SDF3D = abs(SDF3D);

maxx = max(SDF3D,[], "all");
minn = min(SDF3D,[], "all");

% in = SDF3D(SDF3D < 0);
% out = SDF3D(SDF3D >= 0);

in(in >= 0) = minn;
out(out < 0) = 0;

max_in = max(in,[], "all");
min_in = min(in,[], "all");
max_out = max(out,[], "all");
min_out = min(out,[], "all");

in = (in-min_in)/(max_in-min_in);
out = (out-min_out)/(max_out-min_out);

% in = im2uint8(in);
% out = im2uint8(out);

filename = 'testAnimated.gif'; % Specify the output file name
for idx = 1:sz(3)
    rgbImage = cat(3, in(:,:,idx), out(:,:,idx), out(:,:,idx) );
    [A,map] = rgb2ind(rgbImage,255);
    if idx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end

% clc; close all; clear all;