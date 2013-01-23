clear;
x = imread('dataset_QVGA_RGB/000005 0 0 180 -0.541544 0.004560 QVGA.png');
x = im2double(x);
y = x;
y(:,:,:)=0;


%{
for i = 1 : size(x,1)
    for j = 1 : size(x,2)
        if x(i,j,1)>110 && x(i,j,1)<130 && x(i,j,2)>130 && x(i,j,2)<150 && x(i,j,3)>70 && x(i,j,3)<90
            x(i,j,:) = 255;
        else
            x(i,j,:) = 0;
        end
    end
end
imshow(x);
%}
binno = size(x,2);
%binno = 10;
binsize = round(size(x,2)/binno);
for i = 1:binno
    for j = 1:size(x,1)
        z1=0;
        z2=0;
        z3=0;
        for k = 1:binsize
            k = k + binsize*(i-1);
            if(j<=size(x,1) && k<=size(x,2))
                z1 = z1 + x(j,k,1);
                z2 = z2 + x(j,k,2);
                z3 = z3 + x(j,k,3);
            end
        end
        z1=z1/binsize;
        z2=z2/binsize;
        z3=z3/binsize;
        %pause;
        %if z1>0.4314 && z1<0.5098 && z2>0.5098 && z2<0.5882 && z3>0.2745 && z3<0.3529
        if z1>0.2041 && z1<0.4041 && z2>0.4269 && z2<0.6269 && z3>0.1982 && z3<0.3982
            %y(j, round(binsize*(i-1) + binsize/2), :) = 1;
            
            %display('Hello');
            %%{
            for a = 1:binsize
                a = a + binsize*(i-1);
                for b = 1:j-15
                    x(b,a,:) = 0;
                end
            end
            %}
            break;
        end
    end
end
imshow(x);
                
                
        
        
