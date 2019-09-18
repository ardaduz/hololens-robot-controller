counter = 1;
for i=1:15:450
    left = imread("left/left" + sprintf('%04d',i) + ".png");
    right = imread("right/right" + sprintf('%04d',i) + ".png");
    
    left = imrotate(left, -90);
    right = imrotate(right, -90);
    
    imwrite(left, "rotated_images/left/left" + sprintf('%04d',counter) + ".bmp", "bmp");
    imwrite(right, "rotated_images/right/right" + sprintf('%04d',counter) + ".bmp", "bmp");
    
    counter = counter + 1;
end