%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read the Hokuyu URG-04LX 2D LIDAR                                 %
% Author: Yu Gu, Modified from Ehab Al Khatib's code                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rangescan] = Read_Hokuyo(S_Hokuyo,rangescan)
%rangescan=zeros(1, 682);
if S_Hokuyo.BytesAvailable>=2134
%    Lidar_data=zeros(1, 2134);                        % initialize the data matrix
%    rangedata=zeros(1, 2110);
    onlyrangedata=zeros(1, 2048);
    encodeddist=zeros(682,3);
    
    Lidar_data =fscanf(S_Hokuyo,'%c',2134);
    
    Lidar_i=find(Lidar_data==Lidar_data(13));
    rangedata=Lidar_data(Lidar_i(3)+1:end-1);
    for j=0:31
        onlyrangedata((64*j)+1:(64*j)+64)=rangedata(1+(66*j):64+(66*j));
    end
    j=0;
    for index=1:floor(numel(onlyrangedata)/3)
        encodeddist(index,:)=[onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
        j=j+1;
    end
    for index=1:size(encodeddist,1)
        rangescan(index)=decodeSCIP(encodeddist(index,:));
    end
end