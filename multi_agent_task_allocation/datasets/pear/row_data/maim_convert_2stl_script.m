%Rajkishan Arikapudi%
%Stavros Vougioukas Lab%
%University of California Davis%
%-----------------------------------%

clc
clear all
clf
close all
%Initializations
Treeselected = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20];% total number of trees
Fruitradius =  [1 1 1 1 1  1  1  1  1  1 0 0  0  0  0  0  0  0  0  0 ]; % 1 represents data collected with Vernier calipers
x=[];
y=[];
z=[];
for counting = 1:20 %number of trees
    counter = Treeselected(counting);
  
    filename2 = strcat('BranchdataTree',int2str(counter),'.csv');
    
    branchdata = load(filename2);
    clear filename2
    
    %Print function to print a tree
    for i=1:size(branchdata,1)
        N=70;
        cyl_color=[.7 .5 0];
        closed=0;
        lines=1;
        FaceLighting = [.7 .5 0];
        figure(counting)
        FaceColor = [.7 .5 0];
        %[Conee,EndPlate1,EndPlate2] = Cone(branchdata(i,1:3),branchdata(i,6:8),[branchdata(i,4) branchdata(i,9)],N, FaceColor,closed,lines);
        Conee = cone2(branchdata(i,1:3),branchdata(i,6:8),[branchdata(i,4) branchdata(i,9)],N, FaceColor,closed,lines);
        x = [x;Conee.XData/100];
        y = [y;Conee.YData/100];
        z = [z;Conee.ZData/100];
        
    end
    name = join(['tree_pear_',num2str(counting),'.STL'])
    surf2stl(name, x,y,z,'ascii');
    x=[];
    y=[];
    z=[];
end
%figure
%plot3(x,y,z)



axis equal
xlabel('x')
ylabel('y')
zlabel('z')
