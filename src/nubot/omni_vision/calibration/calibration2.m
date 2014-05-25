%%
clear 
clc
DistoMarkLine1=load('errortable.txt');
startx =-900-100;
endx   = 900+100;
starty =-600-100;
endy   = 600+100;
xlong  =endx-startx+1;
ylong  =endy-starty+1;
idx=1;
for i=starty:0
    for j=startx:0
        DistoMarkLine((i-starty)*xlong+j-startx+1)= DistoMarkLine1(idx);
        DistoMarkLine((ylong-1-(i-starty))*xlong+j-startx+1)=DistoMarkLine1(idx);
	    DistoMarkLine((i-starty)*xlong+xlong-1-(j-startx)+1)=DistoMarkLine1(idx);
	    DistoMarkLine((ylong-1-(i-starty))*xlong+xlong-1-(j-startx)+1)=DistoMarkLine1(idx);
        idx=idx+1;
    end
end

fieldwidth =  1200 ;
fieldlength = 1800 ;
resolution =  1;
borderwidth = 12;
fmx = fieldwidth/2+borderwidth;
fmy = fieldlength/2+borderwidth;


%%  x y coordinates
linepoints=load('whites.txt'); 

%% get middle position
xmlDoc=xmlread('ROI.xml');
IDArray = xmlDoc.getElementsByTagName('center_point_row');    % the y coordinate
middle(2) =  str2num( char(IDArray.item(0).getFirstChild.getData) )  ;
IDArray = xmlDoc.getElementsByTagName('center_point_column');     
middle(1) =  str2num ( char(IDArray.item(0).getFirstChild.getData) ) ;  % the x coordinate

for number=1:10
%% optimize mapping parameters
%p1,p2: center
%p3,p4: tan-parameters
%p5,p6,p7: orientation
popsize=1000;
T=100;%s
range = [ middle*.96 1.5 .0068 -fieldlength/2  -fieldwidth/2 -0.5;middle*1.04 1.8 .0074 fieldlength/2 fieldwidth/2  0.5]
disp('start mirror parameter optimization...')
options = gaoptimset( 'PopulationSize',popsize,...
    'PopInitRange',range,...
    'Generations',100,...
    'StallGenLimit',100,...
    'TimeLimit',T,...
    'StallTimeLimit',T,...
    'hybridfcn',@fminsearch);
p=[]
p = ga(@(p)costfcn(p,DistoMarkLine, fieldwidth, fieldlength, resolution, borderwidth, linepoints),size(range,2),options)

%% plot results
%relative position in image  linepoints(:,1) x coordinate
X = linepoints(:,1) - p(1); 
Y = linepoints(:,2) - p(2);
phi = atan2(Y,X);
r = p(3)*tan(p(4)*sqrt(X.*X+Y.*Y))*100;


%local to global
sinphi = sin(phi);
cosphi = cos(phi);

x = p(5)+r.*(cosphi*cos(p(7))-sinphi*sin(p(7)));
y = p(6)+r.*(sinphi*cos(p(7))+cosphi*sin(p(7)));

result(:,number)=p;
end

figure(1);
Image1=imread('pic_save2.bmp');
imshow(Image1)
hold on;
plot(linepoints(:,1),linepoints(:,2),'*y')


result=result';
p=mean(result)

%% plot results
%relative position in image  linepoints(:,1) x coordinate
X = linepoints(:,1) - p(1); 
Y = linepoints(:,2) - p(2);
phi = atan2(Y,X);
r = p(3)*tan(p(4)*sqrt(X.*X+Y.*Y))*100;



%local to global
sinphi = sin(phi);
cosphi = cos(phi);

x = p(5)+r.*(cosphi*cos(p(7))-sinphi*sin(p(7)));
y = p(6)+r.*(sinphi*cos(p(7))+cosphi*sin(p(7)));
figure(2)
plot(x,-y,'r*')
hold on;
plot(p(5),-p(6),'.m','markersize',30)
grid minor

xmlFileName = 'mirror_calib.xml'
docNode = com.mathworks.xml.XMLUtils.createDocument('opencv_storage')
docRootNode = docNode.getDocumentElement;    

center_col= docNode.createElement('center_coloum');   
center_col.appendChild(docNode.createTextNode(sprintf('%10.20d',p(1)))) 
docRootNode.appendChild(center_col);  

center_col= docNode.createElement('center_row');   
center_col.appendChild(docNode.createTextNode(sprintf('%10.20d',p(2)))) 
docRootNode.appendChild(center_col); 

center_col= docNode.createElement('para_a');   
center_col.appendChild(docNode.createTextNode(sprintf('%10.20d',p(3)))) 
docRootNode.appendChild(center_col); 

center_col= docNode.createElement('para_b');   
center_col.appendChild(docNode.createTextNode(sprintf('%10.20d',p(4)))) 
docRootNode.appendChild(center_col); 


xmlwrite(xmlFileName,docNode);  
type(xmlFileName);  
