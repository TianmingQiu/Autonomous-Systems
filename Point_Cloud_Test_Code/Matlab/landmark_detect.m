close all; clear; clc;

raw_data = load('02_45degree_step');

angle = [-2.35619:0.004363:2.35619];

x = raw_data .* cos(angle);
y = raw_data .* sin(angle);

plot(x,y,'*');
width = ceil(100 * (max(x) - min(x)))+2;
height = ceil(100 * (max(y) - min(y)))+2;

x_im = int16(ceil(100 * (x - min(x))))+1;
y_im = int16(ceil(100 * (y - min(y))))+1;

im = zeros(width, height);

im(x_im(1),y_im(1)) = 255;
for i = 1:1081
    im(x_im(i),y_im(i)) = 255;
end
im = (im)';
% imshow(im)

% centers = imfindcircles(im,1)

[H,T,R] = hough(im);
% imshow(H,[],'XData',T,'YData',R,...
%            'InitialMagnification','fit');
%xlabel('\theta'), ylabel('\rho');
%axis on, axis normal, hold on;
P  = houghpeaks(H,7,'threshold',ceil(0.1*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));
% plot(x,y,'s','color','white');
SE=strel('square',3);
im2=imdilate(im,SE);
lines = houghlines(im,T,R,P,'FillGap',150,'MinLength',3);
figure, imshow(im2), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',6,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
